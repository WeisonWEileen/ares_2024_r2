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

#include "armor_detector/projection_node.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/key_point3_d.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"
namespace rc_auto_aim
{
ProjectorNode::ProjectorNode(const rclcpp::NodeOptions & options)
: Node("armor_projection_node"),
  box_detection_sub_(this, "/detector/balls"),
  dep_image_sub_(this, "/camera/aligned_depth_to_color/image_raw")
{
  // test
  this->declare_parameter<double>("roll", 0.0);
  this->declare_parameter<double>("pitch", 0.0);
  this->declare_parameter<double>("yaw", 0.0);

  this->declare_parameter<double>("x", 0.0);
  this->declare_parameter<double>("y", 0.0);
  this->declare_parameter<double>("z", 0.0);

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
  keypoint3d_pub_ = this->create_publisher<yolov8_msgs::msg::KeyPoint3DArray>(
    "/detector/keypoint3d", rclcpp::SensorDataQoS());

  sync_ = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(
    MySyncPolicy(200), box_detection_sub_, dep_image_sub_);

  RCLCPP_INFO(this->get_logger(), "Ready to bind topics.");
  sync_->registerCallback(std::bind(
    &ProjectorNode::keypoint_imageCallback, this, std::placeholders::_1, std::placeholders::_2));

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  this->make_camera_roboarm_tranform();
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

  for (auto & box : boxes_msg->detections) {
    yolov8_msgs::msg::KeyPoint3D keypoint3d;
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
    float x = dep_img.at<ushort>(u, v) /1000.f;
    float y = - x * (v - px) / fx;
    float z = - x * (u - py) / fy;

    std::cout << "--------" << std::endl;
    std::cout << "x: " << x << " y: " << y << " z: " << z << 
    std::endl;


    keypoint3d.id = box.class_id;
    keypoint3d.score = box.confidence;
    keypoint3d.point.x = x;
    keypoint3d.point.y = y;
    keypoint3d.point.z = z;
    keypoint3d_array.data.emplace_back(keypoint3d);
  }
  // 3d坐标发布
  keypoint3d_pub_->publish(keypoint3d_array);
}




  void ProjectorNode::make_camera_roboarm_tranform(){
    double roll, pitch, yaw, x, y, z;
    this->get_parameter("roll", roll);
    this->get_parameter("pitch", pitch);
    this->get_parameter("yaw", yaw);
    this->get_parameter("x", x);
    this->get_parameter("y", y);
    this->get_parameter("z", z);

    RCLCPP_INFO(
      this->get_logger(), "Parameters: roll=%f, pitch=%f, yaw=%f, x=%f, y=%f, z=%f", roll, pitch,
      yaw, x, y, z);

    geometry_msgs::msg::TransformStamped t;

    RCLCPP_INFO(this->get_logger(), "Ready to create tf2 static broadcaster.");
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "roboarm_base";
    t.child_frame_id = "cam_realsense";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;

    // t.transform.translation.x = 0.315f;
    // t.transform.translation.y = -0.2f;
    // t.transform.translation.z = 0.315f;
    tf2::Quaternion q;
    // tf2::Quaternion q2;
    q.setRPY(roll, pitch, yaw);
    // q.setRPY(0, 0.7853, 0.7853);
    // q.inverse();

    // q2.setRPY(0, 0.7853, 0);
    // auto q = q1 * q2;

    // q = q.inverse();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
}
}
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rc_auto_aim::ProjectorNode)
