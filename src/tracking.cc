#include "tracking.h"


namespace dhSLAM{

    void OpticalFlowStereo( cv::Mat previous, 
                            cv::Mat current, 
                            std::vector<cv::Point2f> &previous_pts, 
                            std::vector<cv::Point2f> &current_pts,
                            std::vector<cv::Mat> &lDescriptor)
    {
        std::vector<uchar> status;
        cv::Mat err;

        cv::calcOpticalFlowPyrLK(previous, current, previous_pts, current_pts, status, err);


        const int image_x_size_ = previous.cols;
        const int image_y_size_ = previous.rows;

        // remove err point
        int indexCorrection = 0;

        for( int i = 0; i < status.size(); i++)
        {
            cv::Point2f pt = current_pts.at(i- indexCorrection);
            if((pt.x < 0)||(pt.y < 0 )||(pt.x > image_x_size_)||(pt.y > image_y_size_)) status[i] = 0;
            if (status[i] == 0)	
            {
                        
                        previous_pts.erase ( previous_pts.begin() + i - indexCorrection);
                        current_pts.erase (current_pts.begin() + i - indexCorrection);
                        lDescriptor.erase(lDescriptor.begin() + i - indexCorrection);
                        indexCorrection++;
            }

        }
        
        

        // int indexCorrection_ = 0;
        // int ptsNum = previous_pts.size();
        // for(int i = 0; i < ptsNum; i++){
        //     double diff_x = std::fabs(previous_pts[i - indexCorrection_].x - current_pts[i - indexCorrection_].x);
        //     double diff_y = std::fabs(previous_pts[i - indexCorrection_].y - current_pts[i - indexCorrection_].y);
        //     std::cout << diff_x << "  " << diff_y << std::endl;
        //     if(diff_x > 30 || diff_y > 15){
        //         previous_pts.erase ( previous_pts.begin() + i - indexCorrection_);
        //         current_pts.erase (current_pts.begin() + i - indexCorrection_);
        //         indexCorrection_++;           
        //     }
        // }
    }

    void OpticalFlowStereo( cv::Mat previous, 
                            cv::Mat current, 
                            std::vector<cv::Point2f> &previous_pts, 
                            std::vector<cv::Point2f> &current_pts)
    {
        std::vector<uchar> status;
        cv::Mat err;

        cv::calcOpticalFlowPyrLK(previous, current, previous_pts, current_pts, status, err);


        const int image_x_size_ = previous.cols;
        const int image_y_size_ = previous.rows;

        // remove err point
        int indexCorrection = 0;

        for( int i = 0; i < status.size(); i++)
        {
            cv::Point2f pt = current_pts.at(i- indexCorrection);
            if((pt.x < 0)||(pt.y < 0 )||(pt.x > image_x_size_)||(pt.y > image_y_size_)) status[i] = 0;
            if (status[i] == 0)	
            {
                        
                        previous_pts.erase ( previous_pts.begin() + i - indexCorrection);
                        current_pts.erase (current_pts.begin() + i - indexCorrection);
                        indexCorrection++;
            }

        }
    }

    void OpticalFlowTracking(cv::Mat previous, cv::Mat current, std::vector<cv::Point2f> &previous_pts, std::vector<cv::Point2f> &current_pts, std::vector<int> &trackIds)
    {
        std::vector<uchar> status;
        cv::Mat err;

        cv::calcOpticalFlowPyrLK(previous, current, previous_pts, current_pts, status, err);


        const int image_x_size_ = previous.cols;
        const int image_y_size_ = previous.rows;

        // remove err point
        int indexCorrection = 0;

        for( int i = 0; i < status.size(); i++)
        {
            cv::Point2f pt = current_pts.at(i- indexCorrection);
            if((pt.x < 0)||(pt.y < 0 )||(pt.x > image_x_size_)||(pt.y > image_y_size_)) status[i] = 0;
            if (status[i] == 0)	
            {
                        
                        previous_pts.erase ( previous_pts.begin() + i - indexCorrection);
                        current_pts.erase (current_pts.begin() + i - indexCorrection);
                        trackIds.erase (trackIds.begin() + i - indexCorrection);
                        indexCorrection++;
            }

        }

        int indexCorrection_ = 0;
        int ptsNum = previous_pts.size();
        for(int i = 0; i < ptsNum; i++){
            double diff_x = std::fabs(previous_pts[i - indexCorrection_].x - current_pts[i - indexCorrection_].x);
            double diff_y = std::fabs(previous_pts[i - indexCorrection_].y - current_pts[i - indexCorrection_].y);
            // std::cout << diff_x << "  " << diff_y << std::endl;
            if(diff_x > 200 || diff_y > 120){
                previous_pts.erase ( previous_pts.begin() + i - indexCorrection_);
                current_pts.erase (current_pts.begin() + i - indexCorrection_);
                indexCorrection_++;           
            }
        }   
    }


}