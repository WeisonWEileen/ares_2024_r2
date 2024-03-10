#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

int main(int argc, char** argv) {
    // construct the argument parse and parse the arguments
    CommandLineParser parser(argc, argv,
        "{v|video| |path to the (optional) video file}"
        "{b|buffer|64|max buffer size}");

    // global interactive HSV threshold adjustment
    // 233 blue ball threshold
    Scalar yellowLower(104, 57, 122);
    Scalar yellowUpper(118, 255, 255);

    // global interactive Hough circle parameters
    int minDist = 20;
    int param1_ = 50;
    int param2_ = 30;

    // create windows
    namedWindow("Frame");
    namedWindow("HSV_Binary+Canny+Hough");

    // create trackbars
    createTrackbar("Lower H", "Frame", 0, 255, nullptr);
    createTrackbar("Lower S", "Frame", 0, 255, nullptr);
    createTrackbar("Lower V", "Frame", 0, 255, nullptr);
    createTrackbar("Upper H", "Frame", 0, 255, nullptr);
    createTrackbar("Upper S", "Frame", 0, 255, nullptr);
    createTrackbar("Upper V", "Frame", 0, 255, nullptr);
    createTrackbar("minDist", "HSV_Binary+Canny+Hough", 1, 255, nullptr);
    createTrackbar("param1_", "HSV_Binary+Canny+Hough", 100, 600, nullptr);
    createTrackbar("param2_", "HSV_Binary+Canny+Hough", 20, 100, nullptr);

    setTrackbarPos("Lower H", "Frame", yellowLower[0]);
    setTrackbarPos("Lower S", "Frame", yellowLower[1]);
    setTrackbarPos("Lower V", "Frame", yellowLower[2]);
    setTrackbarPos("Upper H", "Frame", yellowUpper[0]);
    setTrackbarPos("Upper S", "Frame", yellowUpper[1]);
    setTrackbarPos("Upper V", "Frame", yellowUpper[2]);
    setTrackbarPos("minDist", "HSV_Binary+Canny+Hough", minDist);
    setTrackbarPos("param1_", "HSV_Binary+Canny+Hough", param1_);
    setTrackbarPos("param2_", "HSV_Binary+Canny+Hough", param2_);

    VideoCapture cap;

    if (!parser.get<String>("video").empty())
        cap.open(parser.get<String>("video"));
    else
        cap.open(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file or capture device." << std::endl;
        return -1;
    }

    // allow the camera or video file to warm up
    waitKey(2000);

    Mat frame, hsv, blurred, mask, edges, circles;

    while (true) {
        yellowLower = Scalar(getTrackbarPos("Lower H", "Frame"),
                             getTrackbarPos("Lower S", "Frame"),
                             getTrackbarPos("Lower V", "Frame"));
        yellowUpper = Scalar(getTrackbarPos("Upper H", "Frame"),
                             getTrackbarPos("Upper S", "Frame"),
                             getTrackbarPos("Upper V", "Frame"));
        minDist = getTrackbarPos("minDist", "HSV_Binary+Canny+Hough");
        param1_ = getTrackbarPos("param1_", "HSV_Binary+Canny+Hough");
        param2_ = getTrackbarPos("param2_", "HSV_Binary+Canny+Hough");

        // grab the current frame
        cap >> frame;

        if (frame.empty())
            break;

        // B_R function
        Mat max_blue;
        reduce(frame, max_blue, 2, REDUCE_MAX);
        Mat blue_mask = (frame.col(0) == max_blue).mul(255);
        imshow("blue_mask", blue_mask);
        Mat masked_image;
        bitwise_and(frame, frame, masked_image, blue_mask);

        // convert to HSV
        cvtColor(masked_image, hsv, COLOR_BGR2HSV);

        // construct a mask for the preset color between lower and higher
        inRange(hsv, yellowLower, yellowUpper, mask);

        // morphological operations
        dilate(mask, mask, Mat(), Point(-1, -1), 5);
        erode(mask, mask, Mat(), Point(-1, -1), 5);

        // Canny edge detection
        Canny(mask, edges, 50, 300);

        // Hough circle detection
        HoughCircles(edges, circles, HOUGH_GRADIENT, 1, minDist, param1_, param2_, 0, 0);

        if (!circles.empty()) {
            int num_circles = circles.size[1];
            std::cout << num_circles << " detected in HSV binary image" << std::endl;
            draw_rect_for_hough(edges, circles);
        } else {
            std::cout << "0 detected in HSV binary image" << std::endl;
        }

        // show the results
        imshow("HSV Binary Circles", mask);
        imshow("HSV_Binary+Canny+Hough", edges);
        imshow("Frame", frame);

        // exit when 'q' key is pressed
        if (waitKey(1) == 'q')
            break;
    }

    return 0;
}
