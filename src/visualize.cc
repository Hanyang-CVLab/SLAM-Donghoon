#include "visualize.h"


namespace dhSLAM{

    cv::Mat DrawKLTmatchLine(cv::Mat image1, cv::Mat image2, std::vector<cv::Point2f> previous_pts, std::vector<cv::Point2f> current_pts)
    {
        cv::Mat MatchImg; 
        cv::hconcat(image1, image2, MatchImg);
        const int rNum = image1.cols;
        std::vector<cv::Point2f> rImgPtsf(current_pts.size());
        for(int i = 0; i < current_pts.size(); i++){
            cv::Point2f pts(current_pts[i].x + rNum, current_pts[i].y);
            rImgPtsf[i] = pts;
        }
        if (MatchImg.channels() < 3) cv::cvtColor(MatchImg, MatchImg, cv::COLOR_GRAY2RGB);
        for(int i = 0; i < previous_pts.size(); i++){
            cv::Scalar randomColor = cv::Scalar(rand() % 255,rand() % 255,rand() % 255);
            cv::line(MatchImg, previous_pts[i], rImgPtsf[i], randomColor, 1);
            cv::circle(MatchImg, previous_pts[i], 3, randomColor, 1);
            cv::circle(MatchImg, rImgPtsf[i], 3, randomColor, 1);
        }

        return MatchImg.clone();
    }

    cv::Mat DrawKLTmatchLine_vertical(cv::Mat image1, cv::Mat image2, std::vector<cv::Point2f> previous_pts, std::vector<cv::Point2f> current_pts)
    {
        cv::Mat MatchImg; 
        cv::vconcat(image1, image2, MatchImg);
        const int rNum = image1.rows;
        std::vector<cv::Point2f> rImgPtsf(current_pts.size());
        for(int i = 0; i < current_pts.size(); i++){
            cv::Point2f pts(current_pts[i].x , current_pts[i].y + rNum);
            rImgPtsf[i] = pts;
        }
        if (MatchImg.channels() < 3) cv::cvtColor(MatchImg, MatchImg, cv::COLOR_GRAY2RGB);
        for(int i = 0; i < previous_pts.size(); i++){
            cv::Scalar randomColor = cv::Scalar(rand() % 255,rand() % 255,rand() % 255);
            cv::line(MatchImg, previous_pts[i], rImgPtsf[i], randomColor, 1);
            cv::circle(MatchImg, previous_pts[i], 3, randomColor, 1);
            cv::circle(MatchImg, rImgPtsf[i], 3, randomColor, 1);
        }

        return MatchImg.clone();
    }
    
}