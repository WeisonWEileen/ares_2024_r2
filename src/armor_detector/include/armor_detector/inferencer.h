#ifndef INFERENCE_H
#define INFERENCE_H

// Cpp native
#include <fstream>
#include <vector>
#include <string>
#include <random>

// OpenCV / DNN / Inference
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "yolov8_msgs/msg/detection.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"

struct Detection
{
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    // cv::Scalar color{};
    cv::Rect box{};
};

class Inference
{
public:
    Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape = {640, 640}, const std::string &classesTxtFile = "", const bool &runWithCuda = true, const bool &debug = true);
    std::vector<Detection> runInference(const cv::Mat & input);
    cv::Mat draw_frame;

  private:
    void loadClassesFromFile();
    void loadOnnxNetwork();
    cv::Mat formatToSquare(const cv::Mat &source);

    std::string modelPath{};
    std::string classesPath{};
    bool cudaEnabled{};

    std::vector<std::string> classes{"red", "blue", "purple", "rim"};

    cv::Size2f modelShape{};

    float modelConfidenceThreshold {0.85};
    float modelScoreThreshold      {0.8};
    float modelNMSThreshold        {0.70};

    bool letterBoxForSquare = true;

    cv::dnn::Net net;

};

#endif // INFERENCE_H
