#include <iostream>
#include <vector>
#include <getopt.h>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "inference.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    std::string projectBasePath = "/home/user/ultralytics"; // Set your ultralytics base path

    bool runOnGPU = true;

    //
    // Pass in either:
    //
    // "yolov8s.onnx" or "yolov5s.onnx"
    //
    // To run Inference with yolov8/yolov5 (ONNX)
    

    // Note that in this example the classes are hard-coded and 'classes.txt' is a place holder.
    Inference inf("2024_1000_pict_640_480.onnx", cv::Size(480, 640), "classes.txt", runOnGPU);


    cv::VideoCapture cap("/dev/cam0");

    if(!cap.isOpened())
        return -1;

    cv::Mat frame;
    int detections;
    while (true)
    {
        cap >> frame;

        // Inference starts here...
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Detection> output = inf.runInference(frame);
        auto end = std::chrono::high_resolution_clock::now();


        // 计算并打印执行时间
        std::chrono::duration<double> diff = end-start;
        std::cout << "Time to run inference: " << diff.count() << " s\n";
        detections = output.size();
        std::cout << "Number of detections:" << detections << std::endl;

        for (int i = 0; i < detections; ++i)
        {
            Detection detection = output[i];

            cv::Rect box = detection.box;
            cv::Scalar color = detection.color;

            // Detection box
            cv::rectangle(frame, box, color, 2);

            // Detection box text
            std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
            cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
            cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

            cv::rectangle(frame, textBox, color, cv::FILLED);
            cv::putText(frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
        }
        // Inference ends here...

        // This is only for preview purposes
        float scale = 0.8;
        cv::resize(frame, frame, cv::Size(frame.cols*scale, frame.rows*scale));
        cv::imshow("Inference", frame);

        if((char)cv::waitKey(30) == 'q') break;
    }
    cap.release();
    cv::destroyAllWindows();
}
