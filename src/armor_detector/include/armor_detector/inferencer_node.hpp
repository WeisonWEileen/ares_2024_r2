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

namespace rc_auto_aim
{
class InferencerNode : public rclcpp::Node
{
    public:
        InferencerNode(const rclcpp::NodeOptions &options);
    private:

        // @brief callback function for image
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

        std::unique_ptr<Inference> initInferencer();

        void detectBalls(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

        void createDebugPublishers();
        void destroyDebugPublishers();

        void visualizeBoxes(cv::Mat & frame, const std::vector<Detection> & output, int size);

        //those detected_ball publisher

        std::unique_ptr<Inference> inferencer_;

        //declaration of timer 

        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        // Image subscrpition
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

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

        //to store the output for each frame
        std::vector<Detection> output;

};
}

#endif