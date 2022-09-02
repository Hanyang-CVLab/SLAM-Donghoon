#pragma once

#include "utils.h"
#include "common.h"


namespace dhSLAM{

    void OpticalFlowStereo( cv::Mat previous, 
                            cv::Mat current, 
                            std::vector<cv::Point2f> &previous_pts, 
                            std::vector<cv::Point2f> &current_pts, 
                            std::vector<cv::Mat> &lDescriptor);

    void OpticalFlowStereo( cv::Mat previous, 
                            cv::Mat current, 
                            std::vector<cv::Point2f> &previous_pts, 
                            std::vector<cv::Point2f> &current_pts);

    void OpticalFlowTracking(   cv::Mat previous, 
                                cv::Mat current, 
                                std::vector<cv::Point2f> &previous_pts, 
                                std::vector<cv::Point2f> &current_pts, 
                                std::vector<int> &trackIds);

}