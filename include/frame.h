#pragma once


#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "common.h"

namespace dhSLAM{


class Frame
{

public:
    
    Frame()
    {}




    ~Frame()
    {}

    Frame(const Frame &tc)
    {
        frameId = tc.frameId;
        img0 = tc.img0;
        img1 = tc.img1;
        ptsId = tc.ptsId;
        pts = tc.pts;
        pose = tc.pose;
    } 

public:

    int frameId;
    cv::Mat img0, img1;
    std::vector<int> ptsId;
    std::vector<cv::Point2d> pts;

    Vector6d pose;
    

};

}