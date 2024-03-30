#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <omp.h>

struct threshold_params {
    int h_up;
    int s_up;
    int v_up;
    int h_down;
    int s_down;
    int v_down;
};

class Detector {
private:
    threshold_params thres;

public:
    Detector(const threshold_params &params)
        : thres(params) {}

    cv::Mat preprocessingImage(const cv::Mat &rgb_image){
        cv::Mat blurred;
        cv::GaussianBlur(rgb_image, blurred, cv::Size(11, 11), 0);
        
        cv::Mat hsv_image;
        cv::cvtColor(blurred, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat mask;
		cv::inRange(hsv_image, cv::Scalar(thres.h_down, thres.s_down, thres.v_down), cv::Scalar(thres.h_up, thres.s_up, thres.v_up),
					mask);

		cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

        return mask;
    }

    //用于过滤掉非 B/R通道最大的像素点
    cv::Mat b_r_max(const cv::Mat &frame){
        //拷贝一份，用于保留debug信息，后续考虑直接用
        cv::Mat img = frame.clone();

        int w = img.cols;
        int h = img.rows;
        uchar * imgPtr = NULL;

        for(int row = 0; row < h ; row++)
        {
            imgPtr = img.data + row*img.step;
            //注意，image.data与image.ptr<uchar>(0)完全等价;
            for(int col = 0; col < w ; col++)
            {
                uchar * curretPtr = imgPtr + col * 3;
                // 如果蓝色通道比其它通道都大，那么就保留，否则都置0
                if ((*curretPtr - *(curretPtr + 1) > 40) & (*curretPtr - *(curretPtr + 2) > 40)) 
                    continue;
                else{
                    *(curretPtr) = 0;
                    *(curretPtr + 1)  = 0;
                    *(curretPtr + 2) = 0;
                }
            }
        }
        return img;
    }
        //没看法完的低级版本呢
        // std::vector<cv::Mat> rgbChannels(3);
        // std::vector<cv::Mat> channels;
        // split(frame,rgbChannels);

        // int rows = frame.rows; // 获取图像的行数
        // int cols = frame.cols; // 获取图像的列数

        // // cv::Mat output = cv::Mat()
        // //后续这里可以写死？

        // //openmp加速 
        // #pragma omp parallel for

        // for (int i = 0; i < rows; i++)
        // {
        //     for (int j = 0; j < cols; j++)
        //     {
        //         if ((rgbChannels[0].at<u_char>(i, j) > rgbChannels[1].at<u_char>(i, j)) & (rgbChannels[0].at<u_char>(i, j) > rgbChannels[2].at<u_char>(i, j)))
        //         {

        //         }
        //     }
        // }



        //老版本
        // for()
        // cv::Mat bgr_planes[3];
        // cv::split(frame, bgr_planes);
        // cv::Mat max_blue = cv::max(cv::max(bgr_planes[0], bgr_planes[1]), bgr_planes[2]);

        // cv::Mat blue_mask = (bgr_planes[0] == max_blue) & (bgr_planes[1] == max_blue) & (bgr_planes[2] == max_blue);
        // blue_mask.convertTo(blue_mask, CV_8U, 255);

        // cv::Mat masked_image;
        // cv::bitwise_and(frame, frame, masked_image, blue_mask);

        // return masked_image;
    // }

    std::vector<std::vector<cv::Point>> find_ball(const cv::Mat &frame){
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(frame.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }

    void draw_circle(cv::Mat &frame, const std::vector<std::vector<cv::Point>> &cnts){
        if (!cnts.empty()) {
            size_t largestContourIndex = 0;
            double maxArea = cv::contourArea(cnts[0]);

            for (size_t i = 1; i < cnts.size(); ++i) {
                double area = cv::contourArea(cnts[i]);
                if (area > maxArea) {
                    maxArea = area;
                    largestContourIndex = i;
                }
            }

            cv::Point2f center; 
            float radius;
            cv::minEnclosingCircle(cnts[largestContourIndex], center, radius);
            cv::Moments M = cv::moments(cnts[largestContourIndex]);
            if (M.m00 != 0) {
                center = cv::Point2f(M.m10 / M.m00, M.m01 / M.m00);
                float radius = maxArea > 10 ? std::sqrt(maxArea / CV_PI) : 0;
                if (radius > 10) {
                    cv::circle(frame, cv::Point(center.x, center.y), static_cast<int>(radius), cv::Scalar(0, 255, 255), 2);
                    cv::circle(frame, cv::Point(center.x, center.y), 5, cv::Scalar(0, 0, 255), -1);
                    std::cout << center << std::endl;
                }
            }
        }
    }
};

int main(int argc, char *argv[])
{

    //默认路径
    std::string camera_path = "/dev/video4";

    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--cam")
        {
            // 如果找到 --camera 参数，则将下一个参数作为相机路径
            if (i + 1 < argc)
            {
                camera_path = argv[i + 1];
            }
            else
            {
                std::cerr << "Error: No path provided after --camera option." << std::endl;
                return 1;
            }
        }
    }

    cv::VideoCapture cap(camera_path);
    if (!cap.isOpened())
	{
		std::cerr << "Error: Unable to open video capture device." << std::endl;
		return 0;
	}

	// Parameters for thresholding
	threshold_params params = {118, 255, 255, 104, 57, 52};
	Detector detector(params);

	cv::Mat frame;
	while (true)
	{
		// Read a new frame from the camera
		cap >> frame;

		if (frame.empty())
		{
			std::cerr << "Error: Unable to read frame from video capture device." << std::endl;
			break;
		}

		// Preprocess the image
        cv::Mat brmax_image = detector.b_r_max(frame);
        cv::Mat processed_image = detector.preprocessingImage(brmax_image);

        // Find contours
		std::vector<std::vector<cv::Point>> cnts = detector.find_ball(processed_image);

		// Draw circles
		detector.draw_circle(frame, cnts);

		// Display the frame
		cv::imshow("Frame", frame);
		cv::imshow("Processed Image", processed_image);
        cv::imshow("Processed Image 02", brmax_image);

        // Press 'q' to exit
		if (cv::waitKey(1 & 0xFF) == 'q')
		{
			break;
		}
	}

	// Release the video capture device
	cap.release();

	// Close all windows
	cv::destroyAllWindows();
	return 0;
}

// #include "opencv2/opencv.hpp"
// #include <iostream>
 
// using namespace cv;
// using namespace std;
 
// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include <string>

// int main()
// {

//     // rtsp地址变量
//     // 一般main 主码流，sub 子码流
//     std::string rtsp1 = "rtsp://admin:SEGJKL@192.168.3.75:554/h264/ch1/sub/av_stream";

//     // std::string rtsp1 = "rtsp://localhost:8554/live1.sdp";


//     // CAP_FFMPEG：使用ffmpeg解码
//     cv::VideoCapture stream1 = cv::VideoCapture("/dev/video0", cv::CAP_V4L2);
//     stream1.set(3, 640);
//     stream1.set(4, 480);


//     if (!stream1.isOpened())
//     {
//         std::cout << "有视频流未打开" << std::endl;
//         return -1;
//     }

//     cv::Mat frame1;


//     // // 使用namedWindow创建窗口，WINDOW_AUTOSIZE：自动调整窗口大小
//     // cv::namedWindow("rtsp_demo", cv::WINDOW_AUTOSIZE);

//     while (true)
//     {
//         if (!stream1.read(frame1))
//         {
//             std::cout << "有视频流未读取" << std::endl;
//             continue;
//         }


//         cv::imshow("rtsp_demo", frame1);

//         if (cv::waitKey(1) == 27)
//         {
//             break;
//         }
//     }

//     return 0;
// }

// #include <iostream>
// #include <opencv2/opencv.hpp>
// using namespace std;

// int width = 640;
// int heigth = 480;
// cv::VideoCapture cap;
// int main()
// {
// 	cap.open(0);
// 	cap.set(cv::CAP_PROP_FRAME_WIDTH, width);    //设置宽度
// 	cap.set(cv::CAP_PROP_FRAME_HEIGHT, heigth);  //设置长度
// 	cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));//视频流格式
// 	cap.set(cv::CAP_PROP_FPS, 30);//帧率 帧/秒

// 	if (!cap.isOpened())
// 	{
// 		cout << "打开失败" << endl;
// 		return 0;
// 	}
// 	cv::Mat frame;
// 	cv::namedWindow("frame");//创建显示窗口


// 	while (1)
// 	{
// 		cap >> frame;//bool judge=cap.read(frame);
// 		if (frame.empty()) break;
//         cout << "打成功" << endl;
// 		cv::imshow("frame", frame);
// 		if (27 == cv::waitKey(33))//"ESC"
// 			break;
// 	}


// 	cv::destroyWindow("frame");
// 	cap.release();
// 	return 0;
// }