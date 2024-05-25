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
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/inferencer.h"
#include "armor_detector/inferencer_node.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/key_point3_d.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"
namespace rc_auto_aim
{
//message_filter 不能使用赋值运算符，因此放在这里赋予话题，初始化
InferencerNode::InferencerNode(const rclcpp::NodeOptions & options)
: Node("armor_detector_node"),
  count_(0)
{
  RCLCPP_INFO(this->get_logger(), "InferencerNode has been started.");
  rgb_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&InferencerNode::imageCallback, this, std::placeholders::_1));
    balls_pub_ = this->create_publisher<yolov8_msgs::msg::DetectionArray>("/detector/balls", 10);
  inferencer_ = initInferencer();
  debug_ = this->declare_parameter("debug", true);
  //发布图像可视化
  result_pub_ = image_transport::create_publisher(this, "/detector/result");

}

void InferencerNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr rgb_img_msg)
{
  // float[] current_pixel_radius;
  detectBalls(rgb_img_msg);
}


//create unique inference object to detect potiential ball
std::unique_ptr<Inference> InferencerNode::initInferencer()
{
  // @TODO 格式化标签读取，参考陈君实现
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/2024_1000_pict_480_640.onnx";
  auto detector =
    std::make_unique<Inference>(model_path, cv::Size(640, 480), "classes.txt", true, true);
  return detector;
}

//run inference to detect
void InferencerNode::detectBalls(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_img_msg)
{
  //头文件对其，后面用于匹配深度图
  //covert ros img msg to cv::Mat
  auto rgb_img = cv_bridge::toCvCopy(rgb_img_msg, "bgr8")->image;
  output_ = inferencer_->runInference(rgb_img);  //这里面已经改变了img
  visualizeBoxes(rgb_img, output_, output_.size());
  boxes_msg_ = convert_to_msg(output_);
  boxes_msg_.header = rgb_img_msg->header;
  //publish
  result_pub_.publish(cv_bridge::CvImage(rgb_img_msg->header, "rgb8", rgb_img).toImageMsg()); 
  balls_pub_->publish(boxes_msg_);
}


  void
  InferencerNode::createDebugPublishers()
{
  result_pub_ = image_transport::create_publisher(this, "/detector/result");
}

void InferencerNode::destroyDebugPublishers() { result_pub_.shutdown(); }

void InferencerNode::visualizeBoxes(
  cv::Mat & frame, const std::vector<Detection> & output_, int size)
{
  for (int i = 0; i < size; ++i) {
    Detection detection = output_[i];

    cv::Rect box = detection.box;

    // Detection box
    cv::rectangle(frame, box, cv::Scalar(200, 200, 200), 2);

    // Detection box text
    std::string classString =
      detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
    cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
    cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

    cv::rectangle(frame, textBox, cv::Scalar(200, 200, 200), cv::FILLED);
    cv::putText(
      frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1,
      cv::Scalar(0, 0, 0), 2, 0);
  }

}  // namespace rc_auto_aim

yolov8_msgs::msg::DetectionArray InferencerNode::convert_to_msg(
  const std::vector<Detection> & detections)
{
  yolov8_msgs::msg::DetectionArray msg;
  // msg.header.stamp = this->now();
  // msg.header.frame_id = "armor_detector";
  // msg.detections.clear();

  for (const auto & detection : detections) {
    yolov8_msgs::msg::Detection detection_msg;
    detection_msg.class_id = detection.class_id;
    detection_msg.confidence = detection.confidence;
    detection_msg.class_name = detection.className;
    detection_msg.bbox.center.position.x = detection.box.x;
    detection_msg.bbox.center.position.y = detection.box.y;
    //detection_msg.bbox.center.theta 这里面还没有利用
    detection_msg.bbox.size.x = detection.box.width;
    detection_msg.bbox.size.y = detection.box.height;
    //计算三维坐标
    msg.detections.emplace_back(detection_msg);
  }

  return msg;
}
}  // namespace rc_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rc_auto_aim::InferencerNode)
