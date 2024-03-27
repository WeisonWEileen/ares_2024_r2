// #include "armor_detector/detector.hpp"
#include "armor_detector/detector.hpp"
#include <vector>
#include <opencv2/opencv.hpp>

namespace rc_auto_aim{
    Detector ::Detector(const threshold_params &params)
        : thres(params) {}

    cv::Mat Detector::preprocessingImage(const cv::Mat & rgb_image){
        cv::Mat blurred;
        cv::GaussianBlur(rgb_image,blurred,cv::Size(11,11),0);
        
        cv::Mat hsv_image;
        cv::cvtColor(blurred,hsv_image,cv::COLOR_BGR2HSV);

        cv::Mat mask;
        cv::inRange(
        hsv_image,
        cv::Scalar(thres.h_up,thres.s_up,thres.v_up),
        cv::Scalar(thres.h_down,thres.s_down,thres.v_down),
        mask);

        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

        return mask;
    }

    cv::Mat Detector::b_r_max(const cv::Mat &frame){
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

    std::vector<std::vector<cv::Point>> Detector::find_ball(const cv::Mat &frame){
       std::vector<std::vector<cv::Point>> contours;
       cv::findContours(frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }

    void Detector::draw_circle(cv::Mat &frame, const std::vector<std::vector<cv::Point>> &cnts){



        if (!cnts.empty()) {
            // find the largest contour in the mask
            size_t largestContourIndex = 0;
            double maxArea = cv::contourArea(cnts[0]);

            //找到最大的轮廓
            for (size_t i = 1; i < cnts.size(); ++i) {
                double area = cv::contourArea(cnts[i]);
                if (area > maxArea) {
                    maxArea = area;
                    largestContourIndex = i;
                }
            }

            //圆心
            cv::Point2f center; 
            float radius;
            // use the largest contour to compute the minimum enclosing circle and centroid
            cv::minEnclosingCircle(cnts[largestContourIndex], center, radius);
            cv::Moments M = cv::moments(cnts[largestContourIndex]);
            if (M.m00 != 0) {
                center = cv::Point2f(M.m10 / M.m00, M.m01 / M.m00);
                // only proceed if the radius meets a minimum size
                float radius = maxArea > 10 ? std::sqrt(maxArea / CV_PI) : 0;
                if (radius > 10) {
                    // draw the circle and centroid on the frame
                    cv::circle(frame, cv::Point(center.x, center.y), static_cast<int>(radius), cv::Scalar(0, 255, 255), 2);
                    cv::circle(frame, cv::Point(center.x, center.y), 5, cv::Scalar(0, 0, 255), -1);
                    std::cout << center << std::endl;
                }
            }
        }
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