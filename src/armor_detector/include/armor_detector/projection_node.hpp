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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "yolov8_msgs/msg/detection.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/key_point3_d.hpp"
#include "yolov8_msgs/msg/key_point3_d_array.hpp"

//color image and depth image synchronize
#include <Eigen/Dense>
#include <camera_info_manager/camera_info_manager.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#define PI 3.1415926f
#define BALL_RADIUS_mm 90
#define Coefficient 0.5
#define BALL_RADIUS_m 0.095

#define TRANSLATION

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


            // void init_TransMatrix();
            // Eigen::Matrix4f cam2arm_trans_ = Eigen::Matrix4f::Identity();

            std::string camera_name_;
            std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
            sensor_msgs::msg::CameraInfo camera_info_msg_;

            // tf2 static broadcaster for camera and roboarm

            // Total tf2 initailization funciton
            void init_tf2();

            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
            // 发布固定的相机相对的机械的坐标关系
            void make_camera_roboarm_tranform();
        
            // tf2 dynamic broadcaster for ball detect
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_dynamic_broadcaster_;

            //buffer to store


            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            // geometry_msgs::msg::Transform cam2arm_tran_;

            tf2::Transform cam2robo_tran_;
            // geometry_msgs::msg::Transform t;
            // tf2::Transform cam2arm_tran_;
            // std::string target_frame_;
            // std::shared_ptr<tf2_filter> tf_filter_


            //球的rviz可视化的信息
            visualization_msgs::msg::Marker ball_marker_;
            visualization_msgs::msg::MarkerArray ball_marker_array_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ball_marker_pub_;

            // 储存球的三维坐标
            yolov8_msgs::msg::KeyPoint3DArray keypoint3d_array_;

            void init_Marker();
            void publishMarkers();
    };
}  // namespace rc_auto_aim

#endif  