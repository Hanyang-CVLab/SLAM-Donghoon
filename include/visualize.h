#pragma once

#include "common.h"
#include "utils.h"


namespace dhSLAM{

    cv::Mat DrawKLTmatchLine(   cv::Mat image1, 
                                cv::Mat image2, 
                                std::vector<cv::Point2f> previous_pts, 
                                std::vector<cv::Point2f> current_pts);

    cv::Mat DrawKLTmatchLine_vertical(  cv::Mat image1, 
                                        cv::Mat image2, 
                                        std::vector<cv::Point2f> previous_pts, 
                                        std::vector<cv::Point2f> current_pts);    

}