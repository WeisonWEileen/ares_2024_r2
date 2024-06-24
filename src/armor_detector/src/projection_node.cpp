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

// STD
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include<iostream>

#include "armor_detector/projection_node.hpp"
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
  RCLCPP_INFO(this->get_logger(), "ProjetionNode has been started.");

  //从yaml中获取外参矩阵 
  init_TransMatrix();
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
    // 深度值
    float z = dep_img.at<ushort>(u, v);
    float x = z * (v - px) / fx;
    float y = z * (u - py) / fy;

    std::cout << "--------" << std::endl;
    std::cout << "x: " << x << " y: " << y << " z: " << z << 
    std::endl;

    // 创建一个齐次坐标
    Eigen::Vector4f point(x,y,z, 1.0);
    // 乘以转换矩阵
    Eigen::Vector4f transformed_point = cam2arm_trans_ * point;

    // 相对于机械臂的坐标系
    keypoint3d.point.x = transformed_point(0);
    keypoint3d.point.y = transformed_point(1);
    keypoint3d.point.z = transformed_point(2);

    std::cout << "X: " << x << " Y: " << y << " Z: " << z << std::endl;
    std::cout << "--------" << std::endl;

    keypoint3d.id = box.class_id;
    keypoint3d.score = box.confidence;
    // keypoint3d.point.x = x;
    // keypoint3d.point.y = y;
    // keypoint3d.point.z = z;
    keypoint3d_array.data.emplace_back(keypoint3d);
  }
  // 3d坐标发布
  keypoint3d_pub_->publish(keypoint3d_array);


}


  void ProjectorNode::init_TransMatrix(){
    RCLCPP_INFO(this->get_logger(), "Ready to create extrinsic matrix.");

    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://armor_detector/config/extrinsic.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    auto raw_matrix = camera_info_msg_.p;

    // 读取12个值
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        cam2arm_trans_(i, j) = raw_matrix[i+j];
      }
    }
    cam2arm_trans_(3, 3) = 1;
    RCLCPP_INFO(this->get_logger(), "------------------------");
    RCLCPP_INFO(this->get_logger(), "Extrinsic matrix is:");
    std::cout << cam2arm_trans_ << std::endl;
    RCLCPP_INFO(this->get_logger(), "------------------------");
  }
}
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rc_auto_aim::ProjectorNode)
