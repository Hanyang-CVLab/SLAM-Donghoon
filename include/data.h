# pragma once

#include <iostream>
#include <fstream>
#include "common.h"
#include "utils.h"

namespace dhSLAM{

    int ReadgtPose(std::string gtpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps);
    int ReadKFPose(std::string KFpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps);

}