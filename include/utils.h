#pragma once

#include<opencv2/core.hpp>
#include "opencv2/opencv.hpp"
// #include <opencv2/core/eigen.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cstdlib>


#include "common.h"

namespace dhSLAM{
    
    ///// Converter /////
    cv::Mat VectorMat2Mat(std::vector<cv::Mat> descriptor);
    std::vector<cv::Mat> Mat2VectorMat(cv::Mat descriptor);
    std::vector<cv::KeyPoint> Point2f2KeyPoint(std::vector<cv::Point2f> pts2f);
    std::vector<cv::Point2f> KeyPoint2Point2f(std::vector<cv::KeyPoint> KeyPoints);
    Eigen::MatrixXf Matf2Eigen(cv::Mat a);
    Eigen::MatrixXd Matd2Eigen(cv::Mat a);
    template <typename T>
    cv::Mat Eigen2Mat(Eigen::Matrix<T, -1, -1> a);


    ///// Transformation /////
        
        // rotation //
        template <typename T>
        Eigen::Matrix<T, 3, 1> ToVec3(Eigen::Matrix<T, 3, 3> rot);
        template <typename T>
        Eigen::Matrix<T, 3, 1> ToVec3(Eigen::Quaternion<T> q);
        template <typename T>
        Eigen::Matrix<T, 3, 3> ToMat33(Eigen::Matrix<T, 3, 1> rod);
        template <typename T>
        Eigen::Quaternion<T> ToQuaternion(Eigen::Matrix<T, 3, 1> rod);
        template <typename T>
        Eigen::Quaternion<T> ToQuaternion(Eigen::Matrix<T, 3, 3> rot);    
    
        // Motion, Projection //
        template <typename T>
        Eigen::Matrix<T, 4, 4> ToMat44(Eigen::Matrix<T, 6, 1> pose);
        template <typename T>
        Eigen::Matrix<T, 4, 4> ToMat44(std::vector<T> pose);
        template <typename T>
        Eigen::Matrix<T, 6, 1> ToVec6(Eigen::Matrix<T, 4, 4> RT);
        template <typename T>
        Eigen::Matrix<T, 6, 1> ToVec6(Eigen::Quaternion<T> q, Eigen::Matrix<T, 3, 1> t);
        template <typename T>
        Eigen::Matrix<T, 6, 1> ToProjection(Eigen::Matrix<T, 6, 1> &pos);
        template <typename T>
        Eigen::Matrix<T, 4, 4> ToProjection(Eigen::Matrix<T, 4, 4> &pos44);
        template <typename T>
        Eigen::Matrix<T, 6, 1> ToMotion(Eigen::Matrix<T, 6, 1> &pos);
        template <typename T>
        Eigen::Matrix<T, 4, 4> ToMotion(Eigen::Matrix<T, 4, 4> &pos44);
        // TODO : flaot?
        cv::Mat ToHomogeneous(cv::Mat projM34);
        template <typename T>
        Eigen::Matrix<T, 4, 4> ToHomogeneous(Eigen::Matrix<T, 3, 4> proj34);

    // For OPENCV Triangulation
    template <typename T>
    cv::Mat ToProj34(Eigen::Matrix<T, 6, 1> pos);
    template <typename T>
    cv::Mat ToProj34(Eigen::Matrix<T, 4, 4> pos44);    
    Eigen::Matrix4d Projd34ToMotion(cv::Mat projM34);
    Eigen::Matrix4f Projf34ToMotion(cv::Mat projM34);

    // Mathmatics
    float VerticalAngle(Eigen::Vector3d p);
    double PointDistance(Eigen::Vector3d p);
    double PointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);
    double CosRaw2(double a, double b, float ang);
    double Rad2Degree(double rad);
    double Ddegree2Rad(double degree);




















// TODO : clean up below code

extern Eigen::Vector3d Origin;
extern Eigen::Vector3d ZVec;
extern Eigen::Matrix3d Iden;

//////////// EuroC ////////////////////

extern double fx, fy, cx, cy;
extern double IntrinsicData[];
      
extern double Cam0ToBodyData[];
extern double Cam1ToBodyData[];
extern cv::Point2d c;


cv::Mat GetK(double* IntrinsicData);
cv::Mat GetKf(float* IntrinsicData);
Eigen::Matrix4d GetCam2Body(double * Cam2BodyData);
Eigen::Matrix4d GetCam1ToCam0(double * Cam2BodyData0, double * Cam2BodyData1);

std::vector<Eigen::Vector3d> Mat3XdToVec3d(Eigen::Matrix3Xd LidarPoints);

double ToAngle(Eigen::Matrix4d LidarRotation);
Eigen::Vector3d ToAxis(Eigen::Matrix4d LidarRotation);

std::vector<cv::Point3d> ToXYZ(cv::Mat &X);


std::vector<float> ReprojectionError(   std::vector<cv::Point3d> WPts, 
                                        std::vector<cv::Point2f> ImgPts, 
                                        Eigen::Matrix4d Pose);

void RemoveMPoutlier(   std::vector<cv::Point3d> &mp, 
                        std::vector<cv::Point2f> &lpts, 
                        std::vector<cv::Point2f> &rpts, 
                        std::vector<cv::Mat> &ldescriptor, 
                        std::vector<cv::Mat> &rdescriptor, 
                        const Vector6d pose);
void RemoveOutlierMatch(    std::vector<cv::Point2f> &lpts, 
                            std::vector<cv::Point2f> &rpts, 
                            std::vector<cv::Mat> &ldescriptor, 
                            std::vector<cv::Mat> &rdescriptor);

Eigen::Matrix4Xf HomogeneousForm(std::vector<cv::Point3d> Wpts);
Eigen::Matrix3Xf HomogeneousForm(std::vector<cv::Point2f> Imgpts);

// cv::Mat rVec6To34ProjMat(Vector6d pose);
}