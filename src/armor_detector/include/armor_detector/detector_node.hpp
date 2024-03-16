#ifndef AMMOR_DETECTOR_HPP_
#define AMMOR_DETECTOR_HPP_

// ROS
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/armor.hpp"

namespace rc_auto_aim
{
class ArmorDetectorNode : public rclcpp::Node
{
    public:
        ArmorDetectorNode(const rclcpp::NodeOptions &options);
    private:
        void publish_target();

        //declaration of timer 
        rclcpp::Publisher<auto_aim_interfaces::msg::Armor>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
};
}

#endif