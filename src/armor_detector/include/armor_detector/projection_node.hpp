#ifndef PROJECTOR_NODE_HPP_
#define PROJECTOR_NODE_HPP_

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
    class ProjectorNode : public rclcpp::Node
    {
        public:
            ProjectorNode(const rclcpp::NodeOptions & options);

        private:
        
          //两个订阅者，一个订阅检测到的目标信息，一个订阅深度图像
          message_filters::Subscriber<yolov8_msgs::msg::DetectionArray> box_detection_sub_;
          message_filters::Subscriber<sensor_msgs::msg::Image> dep_image_sub_;

          //回调的同步策略
          typedef message_filters::sync_policies::ApproximateTime<
            yolov8_msgs::msg::DetectionArray, sensor_msgs::msg::Image>
            MySyncPolicy;

          typedef message_filters::Synchronizer<MySyncPolicy> Sync;
          std::shared_ptr<Sync> sync_;

          //同步器

          //就算坐标的回调函数入口
          void keypoint_imageCallback(
            const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & box_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr & dep_img_msg);
        
          rclcpp::Publisher<yolov8_msgs::msg::KeyPoint3DArray>::SharedPtr keypoint3d_pub_;

        //获取相机rgb和深度对齐之后的内参，用于计算三维坐标，收到一次就关
            std::shared_ptr<sensor_msgs::msg::CameraInfo> aligned_depth_caminfo_;
            rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr aligned_depth_caminfo_sub_;
            void project_to_3d_and_publish(
              const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & boxes_msg,
              const sensor_msgs::msg::Image::ConstSharedPtr & dep_img_msg);
    };
}  // namespace rc_auto_aim

#endif  