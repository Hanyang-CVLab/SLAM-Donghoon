#include "utils.h"

namespace dhSLAM{

// Converter
    cv::Mat VectorMat2Mat(std::vector<cv::Mat> discriptor)
    {
        cv::Mat discriptors(discriptor.front());
        for(size_t i = 1; i < discriptor.size(); i++){
            cv::vconcat(discriptors, discriptor[i], discriptors);
        }

        return discriptors;
    }

    std::vector<cv::Mat> Mat2VectorMat(cv::Mat discriptor)
    {
        std::vector<cv::Mat> discriptors;
        discriptors.resize(discriptor.rows);
        for(int i = 0; i < discriptor.rows; i++){
            discriptors[i] = discriptor.row(i);
        }
        return discriptors;
    }

    std::vector<cv::KeyPoint> Point2f2KeyPoint(std::vector<cv::Point2f> pts2f)
    {
        std::vector<cv::KeyPoint> KeyPoints;
        cv::KeyPoint::convert(pts2f, KeyPoints);

        return KeyPoints;
    }

    std::vector<cv::Point2f> KeyPoint2Point2f(std::vector<cv::KeyPoint> KeyPoints)
    {
        std::vector<cv::Point2f> pts2f;
        cv::KeyPoint::convert(KeyPoints, pts2f);

        return pts2f;
    }

    // float
    Eigen::MatrixXf Matf2Eigen(cv::Mat a)
    {
        Eigen::MatrixXf b(a.rows, a.cols);
        for(int i = 0; i < b.rows(); i++){
            for(int j = 0; j < b.cols(); j++){
                b(i, j) = a.at<float>(i, j);
            }
        }
        return b;
    }

    // double
    Eigen::MatrixXd Matd2Eigen(cv::Mat a)
    {
        Eigen::MatrixXd b(a.rows, a.cols);
        for(int i = 0; i < b.rows(); i++){
            for(int j = 0; j < b.cols(); j++){
                b(i, j) = a.at<double>(i, j);
            }
        }
        return b;
    }

    template <typename T>
    cv::Mat Eigen2Mat(Eigen::Matrix<T, -1, -1> a)
    {
        cv::Mat b(a.rows(), a.cols());
        for(int i = 0; i < b.rows; i++){
            for(int j = 0; j < b.cols; j++){
                b.at<T>(i, j) = a(i, j);
            }
        }
        return b.clone();
    }
    
    template <typename T>
    Eigen::Matrix<T, 3, 1> ToVec3(Eigen::Matrix<T, 3, 3> rot)
    {
        Eigen::AngleAxis<T> rod(rot);
        Eigen::Matrix<T, 3, 1> axis(rod.axis());

        T angle = rod.angle();
        axis *= angle;
        Eigen::Matrix<T, 3, 1> vec3;
        vec3 << axis.x(), axis.y(), axis.z();

        return vec3;
    }
    
    template <typename T>
    Eigen::Matrix<T, 3, 3> ToMat33(Eigen::Matrix<T, 3, 1> rod)
    {
        Eigen::AngleAxis<T> r(rod.norm(), rod.normalized());
        Eigen::Matrix<T, 3, 3> rot = r.toRotationMatrix();

        return rot;
    }
    
    template <typename T>
    Eigen::Matrix<T, 4, 4> ToMat44(Eigen::Matrix<T, 6, 1> pose)
    {
        Eigen::Matrix<T, 3, 1> rod;
        rod << pose[0], pose[1], pose[2];
        Eigen::Matrix<T, 3, 3> rot = ToMat33(rod);

        Eigen::Matrix<T, 4, 4> RT;
        RT <<   rot(0, 0), rot(0, 1), rot(0, 2), pose[3],
                rot(1, 0), rot(1, 1), rot(1, 2), pose[4],
                rot(2, 0), rot(2, 1), rot(2, 2), pose[5],
                0,         0,         0,         1;

        return RT;
    }

    template <typename T>
    Eigen::Matrix<T, 4, 4> ToMat44(std::vector<T> pose)
    {
        Eigen::Matrix<T, 6, 1> vec6;
        vec6 << pose[0], pose[1], pose[2], pose[3], pose[4], pose[5];
        return ToMat44(vec6);
    }

    template <typename T>
    Eigen::Matrix<T, 6, 1> ToVec6(Eigen::Matrix<T, 4, 4> RT)
    {
        Eigen::Matrix<T, 3, 3> R = RT.template block<3, 3>(0, 0);
        Eigen::Matrix<T, 3, 1> rod = ToVec3(R);
        
        Eigen::Matrix<T, 6, 1> Pose;
        Pose << rod.x(), rod.y(), rod.z(), RT(0, 3), RT(1, 3), RT(2, 3);
        return Pose;
    }
    
    template <typename T>
    Eigen::Matrix<T, 6, 1> ToVec6(Eigen::Quaternion<T> q, Eigen::Matrix<T, 3, 1> t)
    {
        Eigen::AngleAxis<T> rod(q);
        Eigen::Matrix<T, 3, 1> r(rod.axis());

        T angle = rod.angle();
        r *= angle;
        Eigen::Matrix<T, 6, 1>  Pose;
        Pose << r.x(), r.y(), r.z(), t.x(), t.y(), t.z();

        return Pose;
    }

    template <typename T>
    Eigen::Matrix<T, 3, 1> ToVec3(Eigen::Quaternion<T> q)
    {
        Eigen::Matrix<T, 3, 3> R(q);
        Eigen::Matrix<T, 3, 1> r = ToVec3(R);
        return r;
    }

    template <typename T>
    Eigen::Quaternion<T> ToQuaternion(Eigen::Matrix<T, 3, 1> rod)
    {
        Eigen::Matrix<T, 3, 3> rot = ToMat33(rod);
        Eigen::Quaternion<T> q(rot);
        return q; 
    }

    template <typename T>
    Eigen::Quaternion<T> ToQuaternion(Eigen::Matrix<T, 3, 3> rot)
    {
        Eigen::Quaternion<T> q(rot);
        return q;
    }

    template <typename T>
    Eigen::Matrix<T, 6, 1> ToProjection(Eigen::Matrix<T, 6, 1> &pos)
    {
        Eigen::Matrix<T, 4, 4> pos44 = ToMat44(pos);
        Eigen::Matrix<T, 4, 4> proj44 = pos44.inverse();
        Eigen::Matrix<T, 6, 1> proj = (proj44);
        return proj;
    }

    template <typename T>
    Eigen::Matrix<T, 4, 4> ToProjection(Eigen::Matrix<T, 4, 4> &pos44)
    {
        Eigen::Matrix<T, 4, 4> proj44 = pos44.inverse();
        return proj44;
    }
    
    template <typename T>
    Eigen::Matrix<T, 6, 1> ToMotion(Eigen::Matrix<T, 6, 1> &proj)
    {
        Eigen::Matrix<T, 4, 4> proj44 = ToMat44(proj);
        Eigen::Matrix<T, 4, 4> pos44 = proj44.inverse();
        Eigen::Matrix<T, 6, 1> pos = ToVec6(pos44);
        return pos;
    }

    template <typename T>
    Eigen::Matrix<T, 4, 4> ToMotion(Eigen::Matrix<T, 4, 4> &proj44)
    {
        Eigen::Matrix<T, 4, 4> pos44 = proj44.inverse();
        return pos44;
    }    
    
    template <typename T>
    cv::Mat ToProj34(Eigen::Matrix<T, 6, 1> pos)
    {
        Eigen::Matrix<T, 4, 4> pos44 = ToMat44(pos);
        // double data[] = {   CamProj(0, 0), CamProj(0, 1), CamProj(0, 2), CamProj(0, 3),
        //                     CamProj(1, 0), CamProj(1, 1), CamProj(1, 2), CamProj(1, 3),
        //                     CamProj(2, 0), CamProj(2, 1), CamProj(2, 2), CamProj(2, 3)};
        // cv::Mat Proj34(3, 4, CV_64F, data);
        cv::Mat projM34 = ToProj34(pos44);
        return projM34.clone();
    }
        
    template <typename T>
    cv::Mat ToProj34(Eigen::Matrix<T, 4, 4> pos44)
    {
        Eigen::Matrix<T, 4, 4> proj44 = pos44.inverse();
        Eigen::Matrix<T, 3, 4> proj34 = proj44.template block<3, 4>(0, 0);
        cv::Mat projM34 = Eigen2Mat(proj34);
        return projM34.clone();        
    }

    cv::Mat ToHomogeneous(cv::Mat projM34)
    {
        cv::Mat projM44;
        cv::Mat Iden = cv::Mat::eye(4, 4, CV_64F);
        cv::vconcat(projM34, Iden.row(3), projM44);
        return projM44;
    }

    template <typename T>
    Eigen::Matrix<T, 4, 4> ToHomogeneous(Eigen::Matrix<T, 3, 4> proj34)
    {
        Eigen::Matrix<T, 1, 4> Iden;
        Iden << 0, 0, 0, 1;
        Eigen::Matrix<T, 4, 4> proj44(proj34.rows() + Iden.rows(), proj34.cols());
        proj44 << proj34, Iden;
        return proj44;
    }    

    Eigen::Matrix4d Projd34ToMotion(cv::Mat projM34)
    {
        cv::Mat projM44 = ToHomogeneous(projM34);
        Eigen::Matrix4d proj44 = Matd2Eigen(projM44);
        return proj44.inverse();      
    }

    Eigen::Matrix4f Projf34ToMotion(cv::Mat projM34)
    {
        cv::Mat projM44 = ToHomogeneous(projM34);
        Eigen::Matrix4f proj44 = Matf2Eigen(projM44);
        return proj44.inverse();      
    }

    float VerticalAngle(Eigen::Vector3d p){
    return atan(p.z() / sqrt(p.x() * p.x() + p.y() * p.y())) * 180 / M_PI;
    }

    double PointDistance(Eigen::Vector3d p){
    return sqrt(p.x()*p.x() + p.y()*p.y() + p.z()*p.z());
    }

    double PointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2){
    return sqrt((p1.x()-p2.x())*(p1.x()-p2.x()) + (p1.y()-p2.y())*(p1.y()-p2.y()) + (p1.z()-p2.z())*(p1.z()-p2.z()));
    }

    double CosRaw2(double a, double b, float ang){
        return sqrt(a * a + b * b - 2 * a * b * cos(ang * M_PI / 180));
    }

    double Rad2Degree(double rad){
        return rad * 180 / M_PI;
    }

    double Ddegree2Rad(double degree){
        return degree * M_PI / 180;
    }


// TODO : clean up below code

//////////////////////////////////////////////////////////
Eigen::Vector3d Origin{0.0, 0.0, 0.0};
Eigen::Vector3d ZVec = Eigen::Vector3d::UnitZ();
Eigen::Matrix3d Iden = Eigen::Matrix3d::Identity();

//////////// EuroC ////////////////////

double fx(435.2046959714599), fy(435.2046959714599), cx(367.4517211914062), cy(252.2008514404297);
double IntrinsicData[] = {   fx, 0.0,cx, 
                            0.0, fy, cy,
                            0.0, 0.0, 1.0}; 
float fx_(435.2046959714599), fy_(435.2046959714599), cx_(367.4517211914062), cy_(252.2008514404297);
float IntrinsicData_[] = {   fx_, 0.0,cx_, 
                            0.0, fy_, cy_,
                            0.0, 0.0, 1.0};

///// Extrinsic /////
// Machine Hall  body(IMU) - cam0 //
double Cam0ToBodyData[] = {0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0};

// Machine Hall  body(IMU) - cam1 //
double Cam1ToBodyData[] = {0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
         0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
        -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
         0.0, 0.0, 0.0, 1.0};


cv::Mat K = GetK(IntrinsicData);
cv::Mat Kf = GetKf(IntrinsicData_);
cv::Point2d c(cx, cy); 








//////////////////////////////////////////////////////////

cv::Mat GetK(double* IntrinsicData)
{
    cv::Mat K(3, 3, CV_64FC1, IntrinsicData);
    return K;
}

cv::Mat GetKf(float* IntrinsicData)
{
    cv::Mat K(3, 3, CV_32FC1, IntrinsicData);
    return K;
}

Eigen::Matrix4d GetCam2Body(double * Cam2BodyData)
{
    Eigen::Matrix4d Cam2Body = Eigen::Map<Eigen::Matrix4d>(Cam2BodyData);
    return Cam2Body.transpose();
}

Eigen::Matrix4d GetCam1ToCam0(double * Cam2BodyData0, double * Cam2BodyData1)
{
    Eigen::Matrix4d Cam0ToBody = Eigen::Map<Eigen::Matrix4d>(Cam2BodyData0);
    Eigen::Matrix4d Cam0ToBody_ = Cam0ToBody.transpose();
    
    Eigen::Matrix4d Cam1ToBody = Eigen::Map<Eigen::Matrix4d>(Cam2BodyData1);
    Eigen::Matrix4d Cam1ToBody_ = Cam1ToBody.transpose();
    
    Eigen::Matrix4d Cam1ToCam0 = Cam0ToBody_.inverse() * Cam1ToBody_;
    return Cam1ToCam0;
}




std::vector<Eigen::Vector3d> Mat3XdToVec3d(Eigen::Matrix3Xd LidarPoints)
{
    std::vector<Eigen::Vector3d> PointCloud(LidarPoints.cols());
    for(int i = 0; i < LidarPoints.cols(); i++){
        PointCloud[i].x() = LidarPoints(0, i);
        PointCloud[i].y() = LidarPoints(1, i);
        PointCloud[i].z() = LidarPoints(2, i);
    }

    return PointCloud;
}

















double ToAngle(Eigen::Matrix4d LidarRotation)
{
    Eigen::Matrix3d rot = LidarRotation.block<3, 3>(0, 0);
    Eigen::AngleAxisd rod(rot);
    double angle = rod.angle();

    return angle;
}

Eigen::Vector3d ToAxis(Eigen::Matrix4d LidarRotation)
{
    Eigen::Matrix3d rot = LidarRotation.block<3, 3>(0, 0);
    Eigen::AngleAxisd rod(rot);
    Eigen::Vector3d Axis = rod.axis();    

    return Axis;
}





std::vector<cv::Point3d> ToXYZ(cv::Mat &X)
{
    std::vector<cv::Point3d> MapPts;
    X.convertTo(X, CV_64F);
    for (int i = 0 ; i < X.cols; i++)
    {
        X.col(i).row(0) = X.col(i).row(0) / X.col(i).row(3);
        X.col(i).row(1) = X.col(i).row(1) / X.col(i).row(3);
        X.col(i).row(2) = X.col(i).row(2) / X.col(i).row(3);
        X.col(i).row(3) = 1;
        MapPts.push_back(cv::Point3d(X.at<double>(0, i), X.at<double>(1, i), X.at<double>(2, i)));
    }

    return MapPts;
}
    



std::vector<float> ReprojectionError(std::vector<cv::Point3d> WPts, std::vector<cv::Point2f> ImgPts, Eigen::Matrix4d Pose)
{
    Eigen::Matrix4Xf WorldPoints = HomogeneousForm(WPts);
    Eigen::Matrix3Xf ImagePoints = HomogeneousForm(ImgPts);

    Eigen::Matrix3Xf ReprojectPoints(3, WorldPoints.cols());
    Pose = Pose.inverse();
    Eigen::Matrix4f Pose_ = Pose.cast<float>();
    Eigen::Matrix<float, 3, 4> PoseRT;
    PoseRT = Pose_.block<3, 4>(0, 0);
    Eigen::MatrixXf K_ = Matf2Eigen(Kf);
    ReprojectPoints = PoseRT * WorldPoints;

    for(int i = 0; i < ReprojectPoints.cols(); i++){
        ReprojectPoints(0, i) /= ReprojectPoints(2, i);
        ReprojectPoints(1, i) /= ReprojectPoints(2, i);
        ReprojectPoints(2, i) /= ReprojectPoints(2, i);
    }
    ReprojectPoints = K_ * ReprojectPoints;

    // for(int i = 0; i < ReprojectPoints.cols(); i++){
    //     std::cout << ReprojectPoints(0, i) << " " << ReprojectPoints(1, i) << " " << ReprojectPoints(2, i) << std::endl;
    // }
    // for(int i = 0; i < ImagePoints.cols(); i++){
    //     std::cout << ImagePoints(0, i) << " " << ImagePoints(1, i) << " " << ImagePoints(2, i) << std::endl;
    // }
    std::vector<float> ReprojectErr(WorldPoints.cols());
    for(int i = 0; i < WorldPoints.cols(); i++){
        ReprojectErr[i] = std::sqrt( (ImagePoints(0, i) - ReprojectPoints(0, i)) * 
                                     (ImagePoints(0, i) - ReprojectPoints(0, i)) + 
                                     (ImagePoints(1, i) - ReprojectPoints(1, i)) *
                                     (ImagePoints(1, i) - ReprojectPoints(1, i)) );
        // std::cout << ReprojectErr[i] << " ";
    }
    // std::cout << std::endl;

    return ReprojectErr;
}










void RemoveMPoutlier(   std::vector<cv::Point3d> &mp, 
                        std::vector<cv::Point2f> &lpts, 
                        std::vector<cv::Point2f> &rpts, 
                        std::vector<cv::Mat> &ldescriptor, 
                        std::vector<cv::Mat> &rdescriptor, 
                        const Vector6d pose)
{
    Eigen::Matrix<double, 4, 1> initCamView;
    initCamView << 0, 0, 1, 0;
    Eigen::Matrix4d camPose = ToMat44(pose);
    
    Eigen::Matrix<double, 4, 1> currCamView_ = camPose * initCamView;
    std::cout << "current view : " << currCamView_.transpose() << std::endl;
    Eigen::Vector3d currCamView;
    currCamView << currCamView_(0), currCamView_(1), currCamView_(2);
    
    // std::vector<cv::Point3d> clone_map_point(mp);
    int mpNum = mp.size();
    int indexCorrection = 0;
        for(int i = 0; i < mpNum; i++){

            Eigen::Vector3d mpView;
            mpView.x() = mp[i - indexCorrection].x - pose[3];
            mpView.y() = mp[i - indexCorrection].y - pose[4];
            mpView.z() = mp[i - indexCorrection].z - pose[5];

                    
            // Eigen::Vector4d vecMP;
            // vecMP << clone_map_point[i].x, clone_map_point[i].y, clone_map_point[i].z, 0;

            if(currCamView.dot(mpView) < 0 || mpView.dot(mpView) > 200)
            {
                mp.erase(mp.begin() + i - indexCorrection);
                lpts.erase(lpts.begin() + i - indexCorrection);
                rpts.erase(rpts.begin() + i - indexCorrection);
                ldescriptor.erase(ldescriptor.begin() + i - indexCorrection);
                rdescriptor.erase(rdescriptor.begin() + i - indexCorrection);
                indexCorrection++;
            }
                
        }

    std::cout << " remove cam back and remain landmark num :  " << mp.size() << std::endl;    
    int indexCorrection_ = 0;
    Eigen::Matrix4d Pose44 = ToMat44(pose);
    std::vector<float> reProjErr = ReprojectionError(mp, lpts, Pose44);
    for(int i = 0; i < reProjErr.size(); i++){
        if(reProjErr[i] > 3.1){
            mp.erase(mp.begin() + i - indexCorrection_);
            lpts.erase(lpts.begin() + i - indexCorrection_);
            rpts.erase(rpts.begin() + i - indexCorrection_);
            ldescriptor.erase(ldescriptor.begin() + i - indexCorrection_);
            rdescriptor.erase(rdescriptor.begin() + i - indexCorrection_);
        }
    }
}
                        
void RemoveOutlierMatch(    std::vector<cv::Point2f> &lpts, 
                            std::vector<cv::Point2f> &rpts, 
                            std::vector<cv::Mat> &ldescriptor, 
                            std::vector<cv::Mat> &rdescriptor)
{
    cv::Mat inlierMask;
    cv::Mat E = cv::findEssentialMat(lpts, rpts, fx, c, cv::RANSAC, 0.99, 1, inlierMask);

    std::vector<cv::Point2f> clone_lpts(lpts);
    std::vector<cv::Point2f> clone_rpts(rpts);
    std::vector<cv::Mat> clone_ldes(ldescriptor);
    std::vector<cv::Mat> clone_rdes(rdescriptor);
    
    lpts.clear();
    rpts.clear();
    ldescriptor.clear();
    rdescriptor.clear();
    
    for(int i = 0; i < inlierMask.rows; i++){
      if(inlierMask.at<bool>(i, 0) == 1){
        
        lpts.push_back(clone_lpts[i]);
        rpts.push_back(clone_rpts[i]);
        ldescriptor.push_back(clone_ldes[i]);
        rdescriptor.push_back(clone_rdes[i]);
      }
    }    
}

    Eigen::Matrix4Xf HomogeneousForm(std::vector<cv::Point3d> Wpts)
    {
        Eigen::Matrix4Xf WPts(4, Wpts.size());
        for(int i = 0; i < Wpts.size(); i++){
            WPts(0, i) = (float)Wpts[i].x;
            WPts(1, i) = (float)Wpts[i].y;
            WPts(2, i) = (float)Wpts[i].z;
            WPts(3, i) = 1.0f;
        }
        return WPts;   
    }

    Eigen::Matrix3Xf HomogeneousForm(std::vector<cv::Point2f> Imgpts)
    {
        Eigen::Matrix3Xf ImgPts(3, Imgpts.size());
        for(int i = 0; i < Imgpts.size(); i++){
            ImgPts(0, i) = Imgpts[i].x;
            ImgPts(1, i) = Imgpts[i].y;
            ImgPts(2, i) = 1.0f;
        }
        return ImgPts;   
    }
}


// cv::Mat rVec6To34ProjMat(Vector6d pose)
// {
//     Eigen::Matrix4d lCamPose = ToMat44(pose);
//     Eigen::Matrix4d rCamPose = lCamPose * GetCam1ToCam0(Cam0ToBodyData, Cam1ToBodyData);
//     // rCamPose(0,3) = rCamPose(0,3) + 0.5;
//     std::cout << "rpose : " << rCamPose << std::endl;
//     Eigen::Matrix4d CamProj = rCamPose.inverse();
//     double data[] = {   CamProj(0, 0), CamProj(0, 1), CamProj(0, 2), CamProj(0, 3),
//                         CamProj(1, 0), CamProj(1, 1), CamProj(1, 2), CamProj(1, 3),
//                         CamProj(2, 0), CamProj(2, 1), CamProj(2, 2), CamProj(2, 3)};    
//     cv::Mat Proj34(3, 4, CV_64F, data);
//     return Proj34.clone();


// }