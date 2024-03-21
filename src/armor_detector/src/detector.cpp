// #include "armor_detector/detector.hpp"
#include "armor_detector/detector.hpp"
#include <opencv2/opencv.hpp>

namespace rc_auto_aim{
    Detector ::Detector(const threshold_params &params)
        : thres(params) {}

    cv::Mat preprocessingImage(const cv::Mat &input){
        return input;
    }

    cv::Mat b_r_max(const cv::Mat &frame){
        // 计算每个像素BGR值中最大的值
        cv::Mat bgr_planes[3];
        cv::split(frame, bgr_planes);
        cv::Mat max_blue = cv::max(cv::max(bgr_planes[0], bgr_planes[1]), bgr_planes[2]);

        // 创建蓝色掩码，只有蓝色值是最大值的像素才被保留
        cv::Mat blue_mask = (bgr_planes[0] == max_blue) & (bgr_planes[1] == max_blue) & (bgr_planes[2] == max_blue);
        blue_mask.convertTo(blue_mask, CV_8U, 255);

        // 应用掩码到图像
        cv::Mat masked_image;
        cv::bitwise_and(frame, frame, masked_image, blue_mask);

        return masked_image;
    }
}

int main()
{
    // 读取图像
    cv::Mat frame = cv::imread("test.jpg");
    // if (frame.empty())
    // {
    //     std::cerr << "Error: Could not read image." << std::endl;
    //     return -1;
    // }

    // 调用 B_R 函数
    cv::Mat result = rc_auto_aim::b_r_max(frame);

    // 显示结果图像
    cv::imshow("Result", result);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}