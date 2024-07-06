#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
// STD
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rc_detector/projection_node.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/key_point3_d.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"



namespace rc_detector
{
ProjectorNode::ProjectorNode(const rclcpp::NodeOptions & options)
: Node("armor_projection_node"),
  box_detection_sub_(this, "/realsense/results"),
  dep_image_sub_(this, "/camera/aligned_depth_to_color/image_raw")
{
  // test
  this->declare_parameter<double>("roll", 0.0);
  this->declare_parameter<double>("pitch", PI / 4);
  this->declare_parameter<double>("yaw", PI / 4);

  this->declare_parameter<double>("x", 0.256);
  this->declare_parameter<double>("y", -0.2385);
  this->declare_parameter<double>("z", 0.1897);

  // this->declare_parameter<double>("x", 0.1365);
  // this->declare_parameter<double>("y", -0.0185);
  // this->declare_parameter<double>("z", 0.1897);

  RCLCPP_INFO(this->get_logger(), "ProjetionNode has been started.");

  //从yaml中获取外参矩阵
  RCLCPP_INFO(this->get_logger(), "Ready to create camera_info sub.");
  aligned_depth_caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/aligned_depth_to_color/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      aligned_depth_caminfo_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      aligned_depth_caminfo_sub_.reset();
    });
  RCLCPP_INFO(this->get_logger(), "Ready to create keypoint3d publisher.");
  keypoint3d_pub_ = this->create_publisher<yolov8_msgs::msg::KeyPoint3D>(
    "/rc_decision/keypoint3d", rclcpp::SensorDataQoS());

  sync_ = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(
    MySyncPolicy(200), box_detection_sub_, dep_image_sub_);

  RCLCPP_INFO(this->get_logger(), "Ready to bind topics.");
  sync_->registerCallback(std::bind(
    &ProjectorNode::keypoint_imageCallback, this, std::placeholders::_1, std::placeholders::_2));

  init_tf2();
  // init_Marker();

  // 用于发布marker
  none_keypoint3d_msg_.id = -1;
}

void ProjectorNode::keypoint_imageCallback(
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & box_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & dep_img_msg)
{
  project_to_3d_and_publish(box_msg, dep_img_msg);
}

void ProjectorNode::project_to_3d_and_publish(
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & boxes_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & dep_img_msg)
{
  auto dep_img = cv_bridge::toCvCopy(dep_img_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

  //   auto dep_img = cv_bridge::toCvShare(dep_img_msg)->image;
  yolov8_msgs::msg::KeyPoint3DArray keypoint3d_array;

  // int i = 0;

  // 可视化的信息
  keypoint3d_array_.data.clear();
  ball_marker_array_.markers.clear();
  ball_marker_.id = 0;


  //@TODO 其实这里面可不可以并行加速？
  for (auto & box : boxes_msg->detections) {
    // 像素球心坐标(u,v)
    auto v = int(box.bbox.center.position.x + box.bbox.size.x / 2);
    auto u = int(box.bbox.center.position.y + box.bbox.size.y / 2);

    std::cout << "u: " << u << " v: " << v << std::endl;

    auto info_K = this->aligned_depth_caminfo_->k;
    auto px = info_K[2];
    auto py = info_K[5];
    auto fx = info_K[0];
    auto fy = info_K[4];

    // 默认的：z为深度值
    // float z = dep_img.at<ushort>(u, v);
    // float x = z * (v - px) / fx;
    // float y = z * (u - py) / fy;

    //@TODO 是否需要滤波？ 5邻域平均取深度值，卡尔曼滤波？
    //  为了迎合机械臂执行空间的坐标系，x轴向前，y轴向左，z轴向上
    // float x = dep_img.at<ushort>(u, v)/1000.f;

    float x = (dep_img.at<ushort>(u, v) + BALL_RADIUS_mm * Coefficient) / 1000.f;
    // 注意不能直接加球的半径，应该有个倾角
    float y = -x * (v - px) / fx;
    float z = -x * (u - py) / fy;

    float length = sqrt(x * x + y * y + z * z);
    float scale = (BALL_RADIUS_m + length) / length;

    x *= scale;
    y *= scale;
    z *= scale;

    //rviz可视化frame
    geometry_msgs::msg::TransformStamped trans;
    // 求出球的坐标点
    geometry_msgs::msg::PointStamped ps;

    // @TODO 滤波器
    // geometry_msgs::msg::Point cam_point;
    // geometry_msgs::msg::Point arm_point;
    // tf2::Transform tf2_transform;
    // tf2::fromMsg(t, tf2_transform);

    tf2::Vector3 cam_point_tf2(x, y, z);
    tf2::Vector3 arm_point_tf2 = cam2robo_tran_ * cam_point_tf2;

    // RCLCPP_INFO(
    //   this->get_logger(), "arm_point: (%f, %f, %f)", arm_point_tf2.x(), arm_point_tf2.y(),
    //   arm_point_tf2.z());
    yolov8_msgs::msg::KeyPoint3D keypoint3d;
    

    // --------------------------------------------- //
    // 先暂时用着 linear x linear y linear z 作为球的坐标
    // geometry_msgs::msg::twist keypoint3d;
    // keypoint3d.linear.x = arm_point_tf2.x();
    // keypoint3d.linear.y = arm_point_tf2.y();
    // keypoint3d.linear.z = arm_point_tf2.z();
    // -------------------------------------------- //

    // 可视化的之后使用
    // ball_marker_.id++;
    // ball_marker_.pose.position.x = keypoint3d.point.x = arm_point_tf2.x();
    // ball_marker_.pose.position.y = keypoint3d.point.y = arm_point_tf2.y();
    // ball_marker_.pose.position.z = keypoint3d.point.z = arm_point_tf2.z();

    // ball_marker_.id++;
    keypoint3d.point.x = arm_point_tf2.x();
    keypoint3d.point.y = arm_point_tf2.y();
    keypoint3d.point.z = arm_point_tf2.z();

  // //判断有无球吸附成功
  //   if (keypoint3d.point.z  > 0.2){
  //     keypoint3d.attached = true;
  //   }
  //   else{
  //     keypoint3d.attached = false;
  //   }

  RCLCPP_INFO(
        this->get_logger(), "3D: %f, %f, %f", keypoint3d.point.x, keypoint3d.point.y,
        keypoint3d.point.z);


    keypoint3d_array_.data.emplace_back(keypoint3d);
    // ball_marker_array_.markers.emplace_back(ball_marker_);
      //yolov8_msgs 发布
      // keypoint3d.id = box.class_id;
      // // keypoint3d.score = box.confidence;
      // keypoint3d.point.x = x;
      // keypoint3d.point.y = y;
      // keypoint3d.point.z = z;
      // RCLCPP_INFO(this->get_logger(), "Ready to create keypoint3d publisher.");
  }
  // 最简单的直接发布最近的点

  auto closest_keypoint = std::min_element(
    keypoint3d_array_.data.begin(), keypoint3d_array_.data.end(),
    [](const auto & a, const auto & b) {
      return std::pow(a.point.x, 2) + std::pow(a.point.y, 2) <
             std::pow(b.point.x, 2) + std::pow(b.point.y, 2);
    });

  // 3d坐标发布
  if (closest_keypoint != keypoint3d_array_.data.end())
  {
    keypoint3d_pub_->publish(*closest_keypoint);
  }
  // 如果没有识别结果
  else{
    keypoint3d_pub_->publish(none_keypoint3d_msg_);
  }
  // publishMarkers();
}

void ProjectorNode::make_camera_roboarm_tranform()
{
  double roll, pitch, yaw, x, y, z;
  this->get_parameter("roll", roll);
  this->get_parameter("pitch", pitch);
  this->get_parameter("yaw", yaw);
  this->get_parameter("x", x);
  this->get_parameter("y", y);
  this->get_parameter("z", z);

  RCLCPP_INFO(
    this->get_logger(), "The input Param: roll=%f, pitch=%f, yaw=%f, x=%f, y=%f, z=%f", roll, pitch,
    yaw, x, y, z);

  geometry_msgs::msg::Transform t;

  //offset
  t.translation.x = x - 0.03;
  t.translation.y = y;
  t.translation.z = z;

  tf2::Quaternion q;
  // tf2::Quaternion q2;
  q.setRPY(roll, pitch, yaw);

  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();

  tf2::fromMsg(t, cam2robo_tran_);
  // tf2::fromMsg(t.tranform, cam2arm_tran_);
  // tf_static_broadcaster_->sendTransform(t);
}

void ProjectorNode::init_tf2()
{
  // tf2 initialization
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  this->make_camera_roboarm_tranform();
  tf_dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

  // target_frame_ = this->declare_parameter("target_frame", "roboarm_base");
  // tf_filter_ = std::make_shared<tf2_filter>(
  //   armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
  //   this->get_node_clock_interface(), std::chrono::duration<int>(1));
}

void ProjectorNode::init_Marker()
{
  ball_marker_.ns = "armors";
  ball_marker_.action = visualization_msgs::msg::Marker::ADD;
  ball_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  ball_marker_.scale.x = 0.05;
  ball_marker_.scale.z = 0.125;
  ball_marker_.color.a = 1.0;
  ball_marker_.color.g = 0.5;
  ball_marker_.color.b = 1.0;
  // 球的转向没有什么意义
  ball_marker_.pose.orientation.x = 0.0;
  ball_marker_.pose.orientation.y = 0.0;
  ball_marker_.pose.orientation.z = 0.0;
  ball_marker_.pose.orientation.w = 1.0;
  ball_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  ball_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);
}

void ProjectorNode::publishMarkers()
{
  // @TODO没懂这里的逻辑
  using Marker = visualization_msgs::msg::Marker;
  ball_marker_.action = keypoint3d_array_.data.empty() ? Marker::DELETE : Marker::ADD;
  ball_marker_array_.markers.emplace_back(ball_marker_);
  ball_marker_pub_->publish(ball_marker_array_);
}

void publish_Marker() {}
}  // namespace rc_detector
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rc_detector::ProjectorNode)
