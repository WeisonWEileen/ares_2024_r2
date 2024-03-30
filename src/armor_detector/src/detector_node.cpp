#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "armor_detector/detector_node.hpp"
#include "armor_detector/detector.hpp"

namespace rc_auto_aim
{
    ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options) : Node("armor_detector_node"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode has been started.");

        detector_ = initDetector();
        debug_ = true;
        if (debug_)
        {
            createDebugPublishers();
        }

            // 创建球的目标发布者节点
        balls_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>("/detector/balls", rclcpp::SensorDataQoS());

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
        //     // 创建发布器,设置Node和默认值
        //     publisher_ = this->create_publisher<auto_aim_interfaces::msg::Target>("/tracker/target", 10); // 设置发布频率
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ArmorDetectorNode::publish_target, this));
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



void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg){
    // float[] current_pixel_radius;
    detectArmors(img_msg);

}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
    Detector::threshold_params params = {
        .h_up = 118,
        .s_up = 255,
        .v_up = 255,
        .h_down = 104,
        .s_down = 57,
        .v_down = 52};

    auto detector = std::make_unique<Detector>(params);
    return detector;
}

ball_target ArmorDetectorNode::detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

    cv::Mat b_r_img;
    b_r_img = detector_->b_r_max(img);

    cv::Mat binary_img;
    binary_img = detector_->preprocessImage(b_r_img);

    std::vector<std::vector<cv::Point>> counters;
    counters = detector_->find_ball(binary_img);

    //返回x，y，半径
    ball_target ball = detector_->draw_circle(img, counters);

    auto_aim_interfaces::msg::Target target;
    target.header.stamp = img_msg->header.stamp;
    target.header.frame_id = img_msg->header.frame_id;

    target.center_pixel_point.x = ball.u;
    target.center_pixel_point.y = ball.v;
    target.radius = ball.r;

    balls_pub_->publish(target);

    // 发布视频的debug信息，请注意，这部分的占用的资源可能非常大，在比赛场上请不要使用


    auto final_time = this->now();
    auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");
    if (debug_)
    {

        br_max_img_pub_.publish(
            cv_bridge::CvImage(img_msg->header, "mono8", b_r_img).toImageMsg());

        binary_img_pub_.publish(
            cv_bridge::CvImage(img_msg->header, "mono8",binary_img).toImageMsg());
    }
    return ball;
}

void ArmorDetectorNode::createDebugPublishers()
{
    br_max_img_pub_ = image_transport::create_publisher(this, "/detector/br_max_img");

    binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
}

void ArmorDetectorNode::destroyDebugPublishers()
{
    br_max_img_pub_.shutdown();
    binary_img_pub_.shutdown();
}

}// namespace rc_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rc_auto_aim::ArmorDetectorNode)
