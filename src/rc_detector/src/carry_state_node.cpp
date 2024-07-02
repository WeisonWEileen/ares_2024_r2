// @TODO 红蓝方条件编译

#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// 最大的宽高
#define IMAGE_HEIGHT 480
#define IMAGE_WIDTH  640

// using namespace std;
// using namespace cv;

class CarrystateNode : public rclcpp::Node
{
public:
  CarrystateNode() : Node("rc_carrtstate_node")
  {
    getParam();

    carrayState_pub_ = this->create_publisher<std_msgs::msg::Bool>("/rc_decision/carry_state", 10);

    camera.open(camera_id_);  //设备驱动号
    if (!camera.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
      rclcpp::shutdown();
    }
    
    // 设置摄像头的参数，目前应该不怎么用得到
    // camera.set(CAP_PROP_FRAME_WIDTH, 640);
    // camera.set(CAP_PROP_FRAME_HEIGHT, 480);
    // camera.set(CAP_PROP_FPS, 30);
    // camera.set(CAP_PROP_EXPOSURE, 1.0);
    // camera.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(fps_),  // roughly 20 FPS
      std::bind(&CarrystateNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    cv::Mat frame;
    camera >> frame;

    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Captured empty frame!");
      return;
    }
    //
    RCLCPP_DEBUG(
      this->get_logger(), "ROI values: %ld, %ld, %ld, %ld", roi_[0], roi_[1], roi_[2], roi_[3]);

    cv::Mat ROI = frame(cv::Range(roi_[0], roi_[2]), cv::Range(roi_[1], roi_[3]));
    count_rb_max(ROI);

    cv::imshow("frame", frame);
    cv::imshow("roi", ROI);

    if (cv::waitKey(30) > 0) {
      RCLCPP_INFO(this->get_logger(), "Frame size: %d x %d", frame.rows, frame.cols);
      camera.release();
      ROI.release();
      rclcpp::shutdown();
    }
  }

  void count_rb_max(const cv::Mat & frame) {
    int count_ = 0;
    uchar * imgPtr = NULL;

    for (int row = 0; row < roi_height_; row++) {
      imgPtr = frame.data + row * frame.step;
      for (int col = 0; col < roi_width_; col++) {
        // do something
        uchar * curretPtr = imgPtr + col * 3;
        if ((*curretPtr - *(curretPtr + 1) > 40) & (*curretPtr - *(curretPtr + 2) > 40))
          count_++;
      }
    }
    RCLCPP_INFO(this->get_logger(), "the blue counter is %d", count_);
    
    std_msgs::msg::Bool msg;
    if (count_ > thres_) 
    {
      msg.data = true;
      carrayState_pub_->publish(msg);
    }
    else{
      msg.data = false;
      carrayState_pub_->publish(msg);

    }
  }

  void getParam(){

    this->declare_parameter<std::string>("camera_port", "/dev/cam0");
    camera_id_ = this->get_parameter("camera_port").as_string();

    this->declare_parameter<std::vector<int>>("roi", {0, 0, IMAGE_HEIGHT, IMAGE_WIDTH});
    roi_ = this->get_parameter("roi").as_integer_array();
    roi_height_ = roi_[2]-roi_[0];
    roi_width_ = roi_[3]-roi_[1];

    this->declare_parameter<int>("thres", 500);
    thres_ = this->get_parameter("thres").as_int();

    this->declare_parameter<int>("fps", 20);
    fps_ = this->get_parameter("fps").as_int();

      //检查node参数是否输入正确
    if (roi_.size() < 4)
    {
      RCLCPP_ERROR(this->get_logger(), "ROI array is too short! %ld", roi_.size());
      return;
    }

    if (roi_[0] < 0 || roi_[1] < 0 || roi_[2] > IMAGE_HEIGHT || roi_[3] > IMAGE_WIDTH) {
      RCLCPP_ERROR(this->get_logger(), "ROI is out of frame bounds!");
      return;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr carrayState_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture camera;
  std::string camera_id_;
  std::vector<long int> roi_;
  int roi_width_;
  int roi_height_;
  int thres_;
  int fps_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarrystateNode>());
  rclcpp::shutdown();
  return 0;
}
