#ifndef AMMOR_DETECTOR_HPP_
#define AMMOR_DETECTOR_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "rc_detector/inferencer.h"
#include "yolov8_msgs/msg/detection.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/key_point3_d.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"

//color image and depth image synchronize
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "rc_detector/common.hpp"
#include "rc_detector/yolov8.hpp"
#include <std_msgs/msg/bool.hpp>

// self defined ros msgs
#include "rc_interface_msgs/srv/init_pos.hpp"

const std::vector<std::string> CLASS_NAMES = {"red", "blue", "purple", "rim"};
const std::vector<std::vector<unsigned int>> COLORS = {
  {2555, 0, 0}, {0, 255, 0}, {128, 128, 0}, {170, 170, 128}};

namespace rc_detector
{
class InferencerNode : public rclcpp::Node
{
public:
  InferencerNode(const rclcpp::NodeOptions & options);

private:
  // 获取参数的接入点
  void getParams();
  // @brief callback function for image



  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_img_msg);
  // 解算3D坐标的回调函数入口
  // void keypoint_imageCallback(
  //   const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & box_msg,
  //   const sensor_msgs::msg::Image::ConstSharedPtr & dep_img_msg);

#ifdef ONNX
  std::unique_ptr<Inference> initInferencer();
#elif defined TENSORRT
  std::unique_ptr<YOLOv8> initInferencer();

#endif

  void detectBalls(const sensor_msgs::msg::Image::ConstSharedPtr & rgb_img_msg);

  void project_to_3d_and_publish(
    const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & boxes_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & dep_img_msg);

  void createDebugPublishers();
  void destroyDebugPublishers();

  void visualizeBoxes(cv::Mat & frame, const std::vector<Detection> & output, int size);

//those detected_ball publisher
#ifdef ONNX
  std::unique_ptr<Inference> inferencer_;
#elif defined TENSORRT
  // 这里实际上已经完全绕过了Inference类，直接使用了Yolov8类
  std::unique_ptr<YOLOv8> inferencer_;
  // 需要使用一个common.hpp中的Objects结构体去储存检测到的目标
  cv::Size size_ = cv::Size{640, 640};
  std::vector<Object> objs_;
  // @TODO 和onnx那边的操作合并

#endif


  // topics
  std::string cam_rgb_topic_;

  //declaration of timer
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  // Image subscrpition
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;

//convert dnn&&nmx output to box
#ifdef ONNX
  yolov8_msgs::msg::DetectionArray convert_to_msg(const std::vector<Detection> & detections);
#elif defined TENSORRT
  yolov8_msgs::msg::DetectionArray convert_to_msg(const std::vector<Object> & detections);
#endif

  image_transport::Publisher result_pub_;

  // ball coordinate publisher to store boxes_msg_
  yolov8_msgs::msg::DetectionArray boxes_msg_;
  // to store the cv format input for each frame
  cv::Mat rgb_img_;
  rclcpp::Publisher<yolov8_msgs::msg::DetectionArray>::SharedPtr balls_pub_;

  //to store the output for each frame
  std::vector<Detection> output_;

  // 用于发布机器人是在1区还是二区，一区是false，二区是true

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr position_;

  // 决策相关

  // 红方还是蓝方
  // 0 false 对应红方，1 true 对应蓝方
  rclcpp::Service<rc_interface_msgs::srv::InitPos>::SharedPtr init_pos_srv_;
  void init_pos_callback(
    const std::shared_ptr<rc_interface_msgs::srv::InitPos::Request> request,
    std::shared_ptr<rc_interface_msgs::srv::InitPos::Response> response);

  // 用于储存一区二区的位置
  uint8_t pos_mode_;
  

  // 用于设置红蓝模式
  uint8_t game_mode_;

  bool calculate_rim_ = 1;
};
}  // namespace rc_detector

#endif