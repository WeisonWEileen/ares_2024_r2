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

#include "armor_detector/inferencer.h"
#include "yolov8_msgs/msg/detection.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/key_point3_d.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"

//color image and depth image synchronize
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

namespace rc_auto_aim
{
class InferencerNode : public rclcpp::Node
{
public:
  InferencerNode(const rclcpp::NodeOptions & options);

private:
  // @brief callback function for image

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr rgb_img_msg);
    // 解算3D坐标的回调函数入口
  void keypoint_imageCallback(
    const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & box_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & dep_img_msg);

  std::unique_ptr<Inference> initInferencer();

  void detectBalls(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_img_msg);

  void project_to_3d_and_publish(
    const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & boxes_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & dep_img_msg);

  void createDebugPublishers();
  void destroyDebugPublishers();

  void visualizeBoxes(cv::Mat & frame, const std::vector<Detection> & output, int size);

  //those detected_ball publisher

  std::unique_ptr<Inference> inferencer_;

  //declaration of timer

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  // Image subscrpition
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;
  message_filters::Subscriber<yolov8_msgs::msg::DetectionArray> box_detection_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> dep_image_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    yolov8_msgs::msg::DetectionArray, sensor_msgs::msg::Image>
    MySyncPolicy;

  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;


  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;

  //convert dnn&&nmx output to box
  yolov8_msgs::msg::DetectionArray convert_to_msg(
    const std::vector<Detection> & detections);

  image_transport::Publisher result_pub_;

  // ball coordinate publisher
  yolov8_msgs::msg::DetectionArray boxes_msg_;
  rclcpp::Publisher<yolov8_msgs::msg::DetectionArray>::SharedPtr balls_pub_;
  rclcpp::Publisher<yolov8_msgs::msg::KeyPoint3DArray>::SharedPtr keypoint3d_pub_;

  //to store the output for each frame
  std::vector<Detection> output_;

  //获取相机rgb和深度对齐之后的内参，用于计算三维坐标，收到一次就关
  std::shared_ptr<sensor_msgs::msg::CameraInfo> aligned_depth_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr aligned_depth_caminfo_sub_;

};
}  // namespace rc_auto_aim

#endif