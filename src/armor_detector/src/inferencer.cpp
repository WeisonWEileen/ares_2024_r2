#include "armor_detector/inferencer.h"
#include "yolov8_msgs/msg/detection.hpp"
#include "yolov8_msgs/msg/bounding_box2_d.hpp"

Inference::Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape, const std::string &classesTxtFile, const bool &runWithCuda,const bool &debug)
{
    modelPath = onnxModelPath;
    modelShape = modelInputShape;
    classesPath = classesTxtFile;
    cudaEnabled = runWithCuda;

    loadOnnxNetwork();
    // loadClassesFromFile(); The classes are hard-coded for this example
}

std::vector<Detection> Inference::runInference(const cv::Mat &input)
{
    cv::Mat modelInput = input;
    if (letterBoxForSquare && modelShape.width == modelShape.height)
        modelInput = formatToSquare(modelInput);

    cv::Mat blob;
    //这玩意一张张地，我真的感觉上很慢很慢，有没有加速的方法？
    cv::dnn::blobFromImage(modelInput, blob, 1.0/255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    std::cout << "Running YOLOv8" << std::endl;
    int rows = outputs[0].size[2];
    int dimensions = outputs[0].size[1];

    outputs[0] = outputs[0].reshape(1, dimensions);
    cv::transpose(outputs[0], outputs[0]);
    
    
    float *data = (float *)outputs[0].data;

    // float x_factor = modelInput.cols / modelShape.width;
    // float y_factor = modelInput.rows / modelShape.height;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i)
    {
        float *classes_scores = data+4;

        cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
        cv::Point class_id;
        double maxClassScore;
        minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

        if (maxClassScore > modelScoreThreshold)
        {
            confidences.push_back(maxClassScore);
            class_ids.push_back(class_id.x);

            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];

            // int left = int((x - 0.5 * w) * x_factor);
            // int top = int((y - 0.5 * h) * y_factor);
            int left = int(x - 0.5 * w);
            int top = int(y - 0.5 * h);

            int width = int(w);
            int height = int(h);

            boxes.push_back(cv::Rect(left, top, width, height));
            //@TODO, 是不是可以换成emplace_back
        }
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    std::vector<Detection> detections{};
    for (unsigned long i = 0; i < nms_result.size(); ++i) {
      int idx = nms_result[i];

      Detection result;
      result.class_id = class_ids[idx];
      result.confidence = confidences[idx];

      //原本的画图在这里省略了

      result.className = classes[result.class_id];
      result.box = boxes[idx];

      detections.push_back(result);
    }

    //直接在这里面把图也画了，就能够直接在detection函数中发布图像了
    return detections;
}

void Inference::loadClassesFromFile()
{
    std::ifstream inputFile(classesPath);
    if (inputFile.is_open())
    {
        std::string classLine;
        while (std::getline(inputFile, classLine))
            classes.push_back(classLine);
        inputFile.close();
    }
}

void Inference::loadOnnxNetwork()
{
    net = cv::dnn::readNetFromONNX(modelPath);
    if (cudaEnabled)
    {
        std::cout << "\nRunning on CUDA" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    else
    {
        std::cout << "\nRunning on CPU" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
}

cv::Mat Inference::formatToSquare(const cv::Mat &source)
{
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

