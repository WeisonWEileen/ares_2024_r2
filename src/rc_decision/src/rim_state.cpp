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
#include <boost/algorithm/string/join.hpp>

#include "armor_detector/yolov8.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/bounding_box2_d.hpp"
#include "yolov8_msgs/msg/key_point3_d.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"

#include "rc_decision/rim_state.hpp"


namespace rc_decision
{
RimStateNode::RimStateNode(const rclcpp::NodeOptions & options) : Node("rimstatenode")
{
  RCLCPP_INFO(this->get_logger(), "RimStateNode has been started.");

  detect_result_sub_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
    "/detector/balls", rclcpp::SensorDataQoS(),
    std::bind(&RimStateNode::detectResultCallback, this, std::placeholders::_1));
}

void RimStateNode::detectResultCallback(
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & detect_result_msg)
{
  rim_state_calculator(detect_result_msg);
}

void RimStateNode::rim_state_calculator(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & detect_result_msg)
{
//for 循环实现
//   for(auto & box boxes_msg->detections){

//     if(box.class_id != 0){
//       continue;
//     }
//   }
  // std::partition实现
//   @TODO 要不要把一些变量变为成员变量，优化效率？

std::fill(rim_state_array_, rim_state_array_ + 10, O_0_E_0);

auto detect_result = std::make_shared<yolov8_msgs::msg::DetectionArray>(*detect_result_msg);
// 分开rim和ball，middle之前的是rim， middle之后的是ball
auto middle = std::partition(
  detect_result->detections.begin(), detect_result->detections.end(),
  [](const yolov8_msgs::msg::Detection & detection) { return detection.class_id == 0; });

// 对rim部分按照center.x从小到大排序
std::sort(
  detect_result->detections.begin(), middle,
  [](const yolov8_msgs::msg::Detection & a, const yolov8_msgs::msg::Detection & b) {
    return a.bbox.center.position.x < b.bbox.center.position.x;
  });

auto num_rim = std::distance(detect_result->detections.begin(), middle);

RCLCPP_INFO(this->get_logger(), "Rim nums: %ld", num_rim);

// rim less than 5，没有识别全

// if (num_rim != 5) {
//   RCLCPP_INFO(this->get_logger(), "Rim less than 5");
//   return;
// }

// Now, [begin, middle) contains detections with class_id == 0 (rim),
// and [middle, end) contains detections with class_id == 1 and 3 (red and blue ).

// Calculate IOU for each pair of detections
// 遍历rim

//   用于打印5个rim状态的字符串
std::vector<std::string> state_strings;

for (auto i = detect_result->detections.begin(); i != middle; ++i) {
  int num_our_side = 0;
  int num_enemy_side = 0;

    // 遍历ball
    for (auto j = middle; j != detect_result->detections.end(); ++j) {
        double iou = bbox_iou(i->bbox, j->bbox);
        if (iou > bbox_thres_) {
        if (j->class_id == 1) {
            num_our_side += 1;
        } else {
            num_enemy_side += 1;
        }
        }
    }
    auto index = std::distance(i, detect_result->detections.begin());
    // RCLCPP_INFO(this->get_logger(), "index : %d", result_string.c_str());

    // 记录的同时并且存储打印数据
    if (num_our_side == 0 && num_enemy_side == 0) {
        rim_state_array_[index] = O_0_E_0;
        state_strings.push_back("O_0_E_0");
    } else if (num_our_side == 1 && num_enemy_side == 0) {
        rim_state_array_[index] = O_1_E_0;
        state_strings.push_back("O_1_E_0");
    } else if (num_our_side == 0 && num_enemy_side == 1) {
        rim_state_array_[index] = O_0_E_1;
        state_strings.push_back("O_0_E_1");
    } else if (num_our_side == 2 && num_enemy_side == 0) {
        rim_state_array_[index] = O_2_E_0;
        state_strings.push_back("O_2_E_0");
    } else if (num_our_side == 1 && num_enemy_side == 1) {
        rim_state_array_[index] = O_1_E_1;
        state_strings.push_back("O_1_E_1");
    } else if (num_our_side == 0 && num_enemy_side == 2) {
        rim_state_array_[index] = O_0_E_2;
        state_strings.push_back("O_0_E_2");
    } else {
        rim_state_array_[index] = FULL;
        state_strings.push_back("O_F_E_F");
    }
}
  std::string result_string =
    boost::algorithm::join(state_strings, " ");  
  RCLCPP_INFO(this->get_logger(), "Rim states: %s", result_string.c_str());
}

float RimStateNode::bbox_iou(
  yolov8_msgs::msg::BoundingBox2D boxA, yolov8_msgs::msg::BoundingBox2D boxB)
{
  // Calculate the coordinates of the intersection rectangle
  float xA = std::max(boxA.center.position.x - boxA.size.x / 2, boxB.center.position.x - boxB.size.x / 2);
  float yA = std::max(boxA.center.position.y - boxA.size.y / 2, boxB.center.position.y - boxB.size.y / 2);
  float xB = std::min(boxA.center.position.x + boxA.size.x / 2, boxB.center.position.x + boxB.size.x / 2);
  float yB = std::min(boxA.center.position.y + boxA.size.y / 2, boxB.center.position.y + boxB.size.y / 2);

  // Compute the area of intersection rectangle
  float interArea = std::max(0.0f, xB - xA) * std::max(0.0f, yB - yA);

  // Compute the area of both the prediction and ground-truth rectangles
  float boxAArea = boxA.size.x * boxA.size.y;
  float boxBArea = boxB.size.x * boxB.size.y;

  // Compute the intersection over union by taking the intersection area and dividing it by the sum of prediction + ground-truth areas - the intersection area
  float iou = interArea / (boxAArea + boxBArea - interArea);

  // Return the intersection over union value
  return iou;
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_decision::RimStateNode)
