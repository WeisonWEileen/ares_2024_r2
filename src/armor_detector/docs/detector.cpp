// // #include "armor_detector/detector.hpp"
// #include "armor_detector/detector.hpp"
// #include <vector>
// #include <opencv2/opencv.hpp>
// #include <omp.h>

// namespace rc_auto_aim{
//     Detector ::Detector(const threshold_params &params)
//         : thres(params) {}

//     cv::Mat Detector::preprocessImage(const cv::Mat &rgb_image)
//     {
//         cv::Mat blurred;
//         cv::GaussianBlur(rgb_image,blurred,cv::Size(11,11),0);
        
//         cv::Mat hsv_image;
//         cv::cvtColor(blurred,hsv_image,cv::COLOR_BGR2HSV);

//         cv::Mat mask;
//         cv::inRange(
//             hsv_image,
//             cv::Scalar(thres.h_down, thres.s_down, thres.v_down),
//             cv::Scalar(thres.h_up, thres.s_up, thres.v_up),
//             mask);

//         cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
//         cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

//         return mask;
//     }
//     // 用于过滤掉非 B/R通道最大的像素点
//     cv::Mat Detector::b_r_max(const cv::Mat &frame)
//     {
//         // 拷贝一份，用于保留debug信息，后续考虑直接用
//         cv::Mat img = frame.clone();

//         int w = img.cols;
//         int h = img.rows;
//         uchar *imgPtr = NULL;

//         // openmp加速,对for循环进行多核并行计算
//         //  #pragma omp parallel for
//         for (int row = 0; row < h; row++)
//         {
//             imgPtr = img.data + row * img.step;
//         // 注意，image.data与image.ptr<uchar>(0)完全等价;
//         // openmp加速,对for循环进行多核并行计算
//         // #pragma omp parallel for //编译时加上 -fopenmp？先不管了，估计开-o3效果差不多
//             for (int col = 0; col < w; col++)
//             {
//                 uchar *curretPtr = imgPtr + col * 3;
//                 // 如果蓝色通道比其它通道都大，那么就保留，否则都置0
//                 if ((*curretPtr - *(curretPtr + 1) > 10) & (*curretPtr - *(curretPtr + 2) > 10))
//                     continue;
//                 else
//                 {
//                     *(curretPtr) = 0;
//                     *(curretPtr + 1) = 0;
//                     *(curretPtr + 2) = 0;
//                 }
//             }
//         }
//         return img;
//     }

//     std::vector<std::vector<cv::Point>> Detector::find_ball(const cv::Mat &frame){
//        std::vector<std::vector<cv::Point>> contours;
//        cv::findContours(frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//         return contours;
//     }

//     ball_target Detector::draw_circle(cv::Mat &frame, const std::vector<std::vector<cv::Point>> &cnts)
//     {
//         ball_target ball;
//         if (!cnts.empty())
//         {
//             // find the largest contour in the mask
//             size_t largestContourIndex = 0;
//             double maxArea = cv::contourArea(cnts[0]);

//             //找到最大的轮廓
//             for (size_t i = 1; i < cnts.size(); ++i) {
//                 double area = cv::contourArea(cnts[i]);
//                 if (area > maxArea) {
//                     maxArea = area;
//                     largestContourIndex = i;
//                 }
//             }

//             //圆心
//             cv::Point2f center; 
//             float radius;
//             // use the largest contour to compute the minimum enclosing circle and centroid
//             cv::minEnclosingCircle(cnts[largestContourIndex], center, radius);
//             cv::Moments M = cv::moments(cnts[largestContourIndex]);
//             if (M.m00 != 0) {
//                 center = cv::Point2f(M.m10 / M.m00, M.m01 / M.m00);
//                 // only proceed if the radius meets a minimum size
//                 float radius = maxArea > 10 ? std::sqrt(maxArea / CV_PI) : 0;
//                 if (radius > 10) {
//                     // draw the circle and centroid on the frame
//                     cv::circle(frame, cv::Point(center.x, center.y), static_cast<int>(radius), cv::Scalar(0, 255, 255), 2);
//                     cv::circle(frame, cv::Point(center.x, center.y), 5, cv::Scalar(0, 0, 255), -1);
//                     std::cout << center << std::endl;

//                     //传递目标信息到函数外，供发布者发布
                    
//                     ball.u = center.x;
//                     ball.v = center.y;
//                     ball.r = radius;
//                 }
//             }
//         }
//         return ball;
//     }

// }

