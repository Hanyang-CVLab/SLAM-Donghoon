#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>
#include <Eigen/Dense>

#include "common.h"
#include "utils.h"
#include "data.h"

using namespace std;



int main(int argc, char *argv[])
{
    if(argc != 4)
    {
        cerr << endl << " Error !! argc params number is not correct ! " << endl;
        return 1;
    }
    
    ///////////////////////////////////////////////////////
    /////////// Load data (Image, Imu, gtPose) ////////////
    
    std::string vocPath = argv[1];
    std::string dataSeqPath = argv[2];
    
    std::string img0Path = dataSeqPath + "/mav0/cam0/data";
    std::string timeStampPath = dataSeqPath + "/mav0/cam0/";
    std::string imuPath = dataSeqPath + "/mav0/imu0/data.csv";
    std::string gtCam0Path =  dataSeqPath + "mav0/cam0/";
    
    // Image Info
    vector<string> imgFileName0;
    vector<double> timeStampsCam;
    
    // Imu Info
    vector<double> timeStampsImu;
    vector<cv::Point3f> gyro, acc;

    // TODO : gtPose info


    dhSLAM::LoadEurocImages(img0Path, timeStampPath, imgFileName0, timeStampsCam);
    dhSLAM::LoadEurocIMU(imuPath, timeStampsImu, acc, gyro);
    // dhSLAM::ReadgtPose();

    int nImages = imgFileName0.size();

    // Main Loop 
    for(size_t i = 0; i < nImages; i++){

    }


    return 0;
}

