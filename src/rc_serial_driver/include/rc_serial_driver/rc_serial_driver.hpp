// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_
#define RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"

namespace rc_serial_driver
{
    class RCSerialDriver : public rclcpp::Node
    {
    public:
        explicit RCSerialDriver(const rclcpp::NodeOptions &options);

        ~RCSerialDriver() override;

    private:
        void getParams();

        void receiveData();

        void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

        void reopenPort();

        void setParam(const rclcpp::Parameter &param);

        void resetTracker();

        // Serial port
        std::unique_ptr<IoContext> owned_ctx_;
        std::string device_name_;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

        // Param client to set detect_colr
        using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
        bool initial_set_param_ = false;
        uint8_t previous_receive_color_ = 0;
        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        ResultFuturePtr set_param_future_;

        // Service client to reset tracker
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

        // Aimimg point receiving from serial port for visualization
        // visualization_msgs::msg::Marker aiming_point_;

        // Broadcast tf from odom to gimbal_link
        double timestamp_offset_ = 0;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

        // For debug usage
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
        // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

        std::thread receive_thread_;
    };
} // namespace rc_serial_driver

#endif // RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_
