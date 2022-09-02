# pragma once

#include <iostream>
#include <fstream>
#include "common.h"
#include "utils.h"

namespace dhSLAM{

    int ReadgtPose(std::string gtpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps);
    int ReadgtPose(const std::string gtpath, std::vector<Vector6d>* poses);
    int ReadKFPose(std::string KFpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps);
    int readCsvGtPose(  std::string gtpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps);
    int FindTimestampIdx(const double a, const std::vector<double> b);             
                    
}