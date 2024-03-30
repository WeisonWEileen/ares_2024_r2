#include <opencv2/opencv.hpp>
#include <omp.h>

using namespace cv;

Mat keepMaxBChannel(const Mat &inputImage)
{
    // 分离通道
    std::vector<Mat> channels;
    split(inputImage, channels);

    // 找到B通道最大值的索引
    int maxIndex = 0;
    double maxValue = 0.0;
    for (int i = 0; i < channels[0].rows; ++i)
    {
        for (int j = 0; j < channels[0].cols; ++j)
        {
            if (channels[0].at<uchar>(i, j) > maxValue)
            {
                maxIndex = i * channels[0].cols + j;
                maxValue = channels[0].at<uchar>(i, j);
            }
        }
    }

    // 创建输出图像
    Mat outputImage = Mat::zeros(inputImage.size(), CV_8UC1);

    // 保留最大B通道值的像素
    outputImage.at<uchar>(maxIndex / outputImage.cols, maxIndex % outputImage.cols) = maxValue;

    return outputImage;
}

// 用于过滤掉非 B/R通道最大的像素点
cv::Mat b_r_max(const cv::Mat &frame)
{
    // 拷贝一份，用于保留debug信息，后续考虑直接用
    cv::Mat img = frame.clone();

    int w = img.cols;
    int h = img.rows;
    uchar *imgPtr = NULL;

    //openmp加速,对for循环进行多核并行计算
    // #pragma omp parallel for
    for (int row = 0; row < h; row++)
    {
        imgPtr = img.data + row * img.step;
        // 注意，image.data与image.ptr<uchar>(0)完全等价;
        // openmp加速,对for循环进行多核并行计算
        #pragma omp parallel for
        for (int col = 0; col < w; col++)
        {
            uchar *curretPtr = imgPtr + col * 3;
            // 如果蓝色通道比其它通道都大，那么就保留，否则都置0
            if ((*curretPtr - *(curretPtr + 1) > 40) & (*curretPtr - *(curretPtr + 2) > 40))
                continue;
            else
            {
                *(curretPtr) = 0;
                *(curretPtr + 1) = 0;
                *(curretPtr + 2) = 0;
            }
        }
    }
    return img;
}

int main()
{
    // 读取图像
    Mat inputImage = imread("picture/ball02.jpg");

    if (inputImage.empty())
    {
        std::cerr << "Could not open or find the image!\n";
        return -1;
    }

    // 调用函数
    Mat outputImage = b_r_max(inputImage);

    // 显示结果
    while(true){
    imshow("Original Image", inputImage);
    imshow("Result Image", outputImage);
    // waitKey(0);

    if (cv::waitKey(1 & 0xFF) == 'q')
    {
        break;
    }
    }

    return 0;

}
