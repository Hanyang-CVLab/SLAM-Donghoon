# pragma once

#include <iostream>
#include <fstream>
#include "common.h"
#include "utils.h"

namespace dhSLAM{

    // Data Load
    void LoadEurocImages(const std::string &strImagePath, const std::string &strPathTimes,
                    std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps);

    void LoadEurocIMU(   const std::string &strImuPath, std::vector<double> &vTimeStamps, 
                    std::vector<cv::Point3f> &vAcc, std::vector<cv::Point3f> &vGyro);

    // For Evaluation
    int ReadgtPose(std::string gtpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps);
    int ReadgtPose(const std::string gtpath, std::vector<Vector6d>* poses);
    int ReadKFPose(std::string KFpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps);
    int readCsvGtPose(  std::string gtpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps);
    int FindTimestampIdx(const double a, const std::vector<double> b);             
                    
}