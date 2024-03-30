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

#include "auto_aim_interfaces/msg/target.hpp"
#include "armor_detector/detector.hpp"

namespace rc_auto_aim
{
class ArmorDetectorNode : public rclcpp::Node
{
    public:
        ArmorDetectorNode(const rclcpp::NodeOptions &options);
    private:

        // @brief callback function for image
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

        std::unique_ptr<Detector> initDetector();
        ball_target detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

        void createDebugPublishers();
        void destroyDebugPublishers();
        void publish_target();

        //those detected_ball publisher
        auto_aim_interfaces::msg::Target balls_msg_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr balls_pub_;

        std::unique_ptr<Detector> detector_;

        //declaration of timer 
        rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        // Image subscrpition
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

        // Debug information
        bool debug_;
        std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
        image_transport::Publisher br_max_img_pub_;
        image_transport::Publisher binary_img_pub_;
};
}

#endif