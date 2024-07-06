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

#include "rc_detector/common.hpp"
#include "rc_detector/inferencer.h"
#include "rc_detector/inferencer_node.hpp"
#include "rc_detector/yolov8.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/key_point3_d.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"

namespace rc_detector
{
//message_filter 不能使用赋值运算符，因此放在这里赋予话题，初始化
InferencerNode::InferencerNode(const rclcpp::NodeOptions & options)
: Node("rc_inferencer_node"), count_(0), pos_mode_(0)
{
  RCLCPP_INFO(this->get_logger(), "InferencerNode has been started.");

  getParams();

  rgb_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    cam_rgb_topic_, rclcpp::SensorDataQoS(),
    std::bind(&InferencerNode::imageCallback, this, std::placeholders::_1));

  init_pos_srv_ = this->create_service<rc_interface_msgs::srv::InitPos>(
    "/rc_decision/init_pose",
    std::bind(
      &InferencerNode::init_pos_callback, this, std::placeholders::_1, std::placeholders::_2));

  balls_pub_ = this->create_publisher<yolov8_msgs::msg::DetectionArray>("/detector/balls", 10);
  inferencer_ = initInferencer();

  //发布图像可视化
  if (debug_) {
    RCLCPP_INFO(this->get_logger(), "ready to create bounding box drawer");
    result_pub_ = image_transport::create_publisher(this, "/detector/result");
  }


}

void InferencerNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_img_msg)
{
  // float[] current_pixel_radius;
  RCLCPP_INFO(this->get_logger(), "Callback");
  detectBalls(rgb_img_msg);
}

#ifdef ONNX
//create unique inference object to detect potiential ball
std::unique_ptr<Inference> InferencerNode::initInferencer()
{
  // @TODO 格式化标签读取，参考陈君实现

  auto pkg_path = ament_index_cpp::get_package_share_directory("rc_detector");
  // auto model_path = pkg_path + "/model/2024_1000_pict_480_640.onnx";
  auto model_path = pkg_path + "/model/6_20_yolov8m.onnx";
  auto detector =
    std::make_unique<Inference>(model_path, cv::Size(640, 480), "classes.txt", true, true);
  return detector;
}
#elif defined TENSORRT
//create unique inference object to detect potiential ball
std::unique_ptr<YOLOv8> InferencerNode::initInferencer()
{
  auto pkg_path = ament_index_cpp::get_package_share_directory("rc_detector");
  auto model_path = pkg_path + "/model/2024_1000.engine";
  auto detector = std::make_unique<YOLOv8>(model_path);
  detector->make_pipe(true);
  return detector;
}

#endif
//run inference to detect
void InferencerNode::detectBalls(const sensor_msgs::msg::Image::ConstSharedPtr & rgb_img_msg)
{
  //头文件对其，后面用于匹配深度图
  //covert ros img msg to cv::Mat

  rgb_img_ = cv_bridge::toCvCopy(rgb_img_msg, "bgr8")->image;

#ifdef ONNX
  auto start = std::chrono::high_resolution_clock::now();
  output_ = inferencer_->runInference(rgb_img_);  //这里面已经改变了img

  // ------  //
  //  0 --> rim
  //  1 --> blue
  //  2 --> purple
  //  3 --> red
  // ------  //


  // 现在这个版本连球框都不识别
  // 如果是蓝方，过滤掉红球
  if(game_mode_){
    output_.erase(
      std::remove_if(
        output_.begin(), output_.end(),
        [](const Detection & detection) { return detection.class_id != 1; }),
      output_.end());
  }
  // 如果是红方，过滤掉蓝球
  else {
    output_.erase(
      std::remove_if(
        output_.begin(), output_.end(),
        [](const Detection & detection) { return detection.class_id != 3; }),
      output_.end());
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end - start;
  std::cout << "Time to run inference: " << diff.count() << " s\n";
  visualizeBoxes(rgb_img_, output_, output_.size());
  boxes_msg_ = convert_to_msg(output_);
  boxes_msg_.header = rgb_img_msg->header;
  //publish
  result_pub_.publish(cv_bridge::CvImage(rgb_img_msg->header, "bgr8", rgb_img_).toImageMsg());
  balls_pub_->publish(boxes_msg_);

#elif defined TENSORRT
// TensorRT版本在还有加上决策
  objs_.clear();
  inferencer_->copy_from_Mat(rgb_img_, size_);
  auto start = std::chrono::system_clock::now();
  inferencer_->infer();
  auto end = std::chrono::system_clock::now();
  inferencer_->postprocess(objs_);
  inferencer_->draw_objects(rgb_img_, rgb_img_, objs_, CLASS_NAMES, COLORS);
  auto tc =
    (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
  printf("cost %2.4lf ms\n", tc);
  boxes_msg_ = convert_to_msg(objs_);
  boxes_msg_.header = rgb_img_msg->header;
  //publish
  result_pub_.publish(cv_bridge::CvImage(rgb_img_msg->header, "bgr8", rgb_img_).toImageMsg());
  balls_pub_->publish(boxes_msg_);
#endif
}

void InferencerNode::createDebugPublishers()
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
      std::to_string(detection.class_id) + ' ' + std::to_string(detection.confidence).substr(0, 4);
    cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 0.2, 2, 0);
    cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

    // cv::rectangle(frame, textBox, cv::Scalar(100, 100, 100), cv::FILLED);
    // cv::putText(
    //   frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1,
    //   cv::Scalar(0, 0, 0), 2, 0);
  }

}  // namespace rc_detector

#ifdef ONNX
yolov8_msgs::msg::DetectionArray InferencerNode::convert_to_msg(
  const std::vector<Detection> & detections)
{
  yolov8_msgs::msg::DetectionArray msg;
  // msg.header.stamp = this->now();
  // msg.header.frame_id = "rc_detector";
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
#elif defined TENSORRT
yolov8_msgs::msg::DetectionArray InferencerNode::convert_to_msg(
  const std::vector<Object> & detections)
{
  yolov8_msgs::msg::DetectionArray msg;
  // msg.header.stamp = this->now();
  // msg.header.frame_id = "rc_detector";
  // msg.detections.clear();

  for (const auto & detection : detections) {
    yolov8_msgs::msg::Detection detection_msg;
    detection_msg.class_id = detection.label;
    detection_msg.confidence = detection.prob;
    // detection_msg.class_name = detection.className;d
    detection_msg.bbox.center.position.x = detection.rect.x;
    detection_msg.bbox.center.position.y = detection.rect.y;
    //detection_msg.bbox.center.theta 这里面还没有利用
    detection_msg.bbox.size.x = detection.rect.width;
    detection_msg.bbox.size.y = detection.rect.height;
    //计算三维坐标
    msg.detections.emplace_back(detection_msg);
  }
  return msg;
}
#endif

void InferencerNode::getParams()
{
  // @TODO vision_bringup里面这里还是获取不了yaml的参数不知道为什么，详情见notion中的ROS

  debug_ = this->declare_parameter<bool>("debug", false);
  debug_ = this->get_parameter("debug").as_bool();

  // cam_rgb_topic_ = this->declare_parameter("cam_rgb_topic", "/image_raw");
  cam_rgb_topic_ = this->declare_parameter<std::string>("cam_rgb_topic", "/camera/color/image_raw");
  cam_rgb_topic_ = this->get_parameter("cam_rgb_topic").as_string();

  pos_mode_ = this->declare_parameter<bool>("game_mode", false);
  pos_mode_ = this->get_parameter("game_mode").as_bool();
}

void InferencerNode::init_pos_callback(
  const std::shared_ptr<rc_interface_msgs::srv::InitPos::Request> request,
  std::shared_ptr<rc_interface_msgs::srv::InitPos::Response> response){
  response->posmode = pos_mode_;
  RCLCPP_WARN_STREAM(this->get_logger(), "send posemod " << pos_mode_);
}

}  // namespace rc_detector

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rc_detector::InferencerNode)
