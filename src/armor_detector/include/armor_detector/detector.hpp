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

        cv::Mat preprocessingImage(const cv::Mat & input);

        // @brief select those pixel which blue/red value are biggest
        // @param: frame 
        cv::Mat b_r_max(const cv::Mat &frame);

        threshold_params thres;
};
}

#endif