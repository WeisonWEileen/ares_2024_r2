/**
 * @author: weison pan
 * @date : 2024-3-20
*/
#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>



// #include 

namespace rc_auto_aim{

  struct ball_target
  {
    float u;
    float v;
    float r;
  };
  class Detector{
    public:
        struct threshold_params
        {
            uint8_t h_up;
            uint8_t s_up;
            uint8_t v_up;
            uint8_t h_down;
            uint8_t s_down;
            uint8_t v_down;
        };

        Detector(const threshold_params &params);
        

        //@brief preprocess image
        cv::Mat preprocessImage(const cv::Mat & input);

        // @brief select those pixel which blue/red value are biggest
        // @param: frame 
        cv::Mat b_r_max(const cv::Mat &frame);

        // @brief to r or b find ball
        std::vector<std::vector<cv::Point>> find_ball(const cv::Mat &frame);

        ball_target draw_circle(cv::Mat &frame, const std::vector<std::vector<cv::Point>> &contours);

        threshold_params thres;
};


}

#endif