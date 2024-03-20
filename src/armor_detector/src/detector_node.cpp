#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "armor_detector/detector_node.hpp"

namespace rc_auto_aim
{
    ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options) : Node("armor_detector_node"), count_(0)
    {
        // 创建发布器,设置Node和默认值
        publisher_ = this->create_publisher<auto_aim_interfaces::msg::Target>("/tracker/target", 10); // 设置发布频率
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ArmorDetectorNode::publish_target, this));
    }
    void ArmorDetectorNode::publish_target()
    {
        auto message = auto_aim_interfaces::msg::Target();
        message.header.stamp = this->now();
        message.header.frame_id = "map";      // 可以根据需要修改
        message.center_pixel_point.x = 100.0; // 修改为您需要的值
        message.center_pixel_point.y = 200.0; // 修改为您需要的值
        message.radius = 50.0;                // 修改为您需要的值
        publisher_->publish(message);
        count_++;
        RCLCPP_INFO(this->get_logger(), "the target publish count is%zu", this->count_);
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rc_auto_aim::ArmorDetectorNode)
