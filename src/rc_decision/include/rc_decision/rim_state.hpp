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

#include "armor_detector/common.hpp"
#include "armor_detector/inferencer.h"
#include "armor_detector/inferencer_node.hpp"
#include "armor_detector/yolov8.hpp"
#include "yolov8_msgs/msg/bounding_box2_d.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/key_point3_d.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"



namespace rc_decision
{
class RimStateNode : public rclcpp::Node
{
public:
  RimStateNode(const rclcpp::NodeOptions & options);

  //  ----   //
  // 枚举单个框的所有可能状态，并且直接对应优先级
  // 敌方：Enemy side
  // 我方：Our side
  //  ----   //
  enum rim_state_var {
    FULL = 0,
    O_0_E_1 = 1,
    O_1_E_0 = 2,
    O_0_E_0 = 3,
    O_2_E_0 = 4,
    O_1_E_1 = 5,
    O_0_E_2 = 6
  };

private:
  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr detect_result_sub_;

  void rim_state_calculator(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & detect_result);
  float bbox_iou(yolov8_msgs::msg::BoundingBox2D boxA, yolov8_msgs::msg::BoundingBox2D boxB);
  float bbox_thres_;

  //对5个框存储优先级的数组,初始化为O_0_E_0
  rim_state_var rim_state_array_[5] = {
    O_0_E_0, 
    O_0_E_0, 
    O_0_E_0, 
    O_0_E_0, 
    O_0_E_0
    };

  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr detect_result_sub_;
  rclcpp::Publisher<std::msgs::msg::String>::SharedPtr rim_state_pub_;

  void detectResultCallback(
    const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & detect_result_msg);
};
}