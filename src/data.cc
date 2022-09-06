#include "data.h"
// #include "common.h"


using namespace std;
namespace dhSLAM{
    
    void LoadEurocImages(const string &strImagePath, const string &strPathTimes,
                    vector<string> &vstrImages, vector<double> &vTimeStamps)
    {
        ifstream fTimes;
        fTimes.open(strPathTimes.c_str());
        vTimeStamps.reserve(5000);
        vstrImages.reserve(5000);
        while(!fTimes.eof())
        {
            string s;
            getline(fTimes,s);
            if(!s.empty())
            {
                stringstream ss;
                ss << s;
                vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
                double t;
                ss >> t;
                vTimeStamps.push_back(t/1e9);

            }
        }
    }

    void LoadEurocIMU(   const string &strImuPath, vector<double> &vTimeStamps, 
                    vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
    {
        ifstream fImu;
        fImu.open(strImuPath.c_str());
        vTimeStamps.reserve(5000);
        vAcc.reserve(5000);
        vGyro.reserve(5000);

        while(!fImu.eof())
        {
            string s;
            getline(fImu,s);
            if (s[0] == '#')
                continue;

            if(!s.empty())
            {
                string item;
                size_t pos = 0;
                double data[7];
                int count = 0;
                while ((pos = s.find(',')) != string::npos) {
                    item = s.substr(0, pos);
                    data[count++] = stod(item);
                    s.erase(0, pos + 1);
                }
                item = s.substr(0, pos);
                data[6] = stod(item);

                vTimeStamps.push_back(data[0]/1e9);
                vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
                vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
            }
        }
    }

    int ReadgtPose(std::string gtpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps)
    {
        std::ifstream gtFile(gtpath, std::ifstream::in);
        if(!gtFile.is_open()){
            std::cout << " gtpose file failed to open " << std::endl;
            return EXIT_FAILURE;
        }

        std::string line;
        while(std::getline(gtFile, line)){
            std::string value;
            std::vector<std::string> values;

            std::stringstream ss(line);
            while(std::getline(ss, value, ' '))
                values.push_back(value);
            
            Vector6d pose;
            pose << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]), std::stod(values[4]), std::stod(values[5]), std::stod(values[6]);
            poses->push_back(pose);
            timeStamps->push_back(std::stod(values[0])/1e9);
        }       

    }

    int ReadgtPose(const std::string gtpath, std::vector<Vector6d>* poses)
    {
        std::ifstream gtFile(gtpath, std::ifstream::in);
        if(!gtFile.is_open()){
            std::cout << " gtpose file failed to open " << std::endl;
            return EXIT_FAILURE;
        }

        std::string line;
        while(std::getline(gtFile, line)){
            std::string value;
            std::vector<std::string> values;

            std::stringstream ss(line);
            while(std::getline(ss, value, ' '))
                values.push_back(value);
            
            Vector6d pose;
            pose << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]), std::stod(values[4]), std::stod(values[5]), std::stod(values[6]);
            poses->push_back(pose);
        }       

    }

    int ReadKFPose(std::string KFpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps)
    {
        std::ifstream KFfile(KFpath, std::ifstream::in);
        if(!KFfile.is_open()){
            std::cout << " KFpose file failed to open " << std::endl;
                return EXIT_FAILURE;
        }

            std::string line;
            while(std::getline(KFfile, line)){
                std::string value;
                std::vector<std::string> values;

                std::stringstream ss(line);
                while(std::getline(ss, value, ' '))
                    values.push_back(value);
                
                Eigen::Quaterniond q;
                q.x() = std::stod(values[4]);
                q.y() = std::stod(values[5]);
                q.z() = std::stod(values[6]);
                q.w() = std::stod(values[7]);

                Eigen::Vector3d t;
                t << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]);

                // Vector6d pose = ToVec6(q, t); 
                // poses->push_back(pose);
                // timeStamps->push_back(std::stod(values[0]));
            }     
    }

    int readCsvGtPose(std::string gtpath, std::vector<Vector6d>* poses, std::vector<double>* timeStamps)
    {
        std::ifstream gtFile(gtpath, std::ifstream::in);
        if(!gtFile.is_open()){
            std::cout << " gtpose file failed to open " << std::endl;
            return EXIT_FAILURE;
        }

        int lineNum = 0;
        std::string line;
        while(std::getline(gtFile, line)){
            if(lineNum == 0){
                lineNum++;
                continue;
            }
            std::string value;
            std::vector<std::string> values;

            std::stringstream ss(line);
            while(std::getline(ss, value, ','))
                values.push_back(value);
            
            Eigen::Quaterniond q;
            q.x() = std::stod(values[5]);
            q.y() = std::stod(values[6]);
            q.z() = std::stod(values[7]);
            q.w() = std::stod(values[4]);

            Eigen::Vector3d t;
            t << std::stod(values[1]), std::stod(values[2]),std::stod( values[3]);
            // Vector6d Pose = ToVec6(q, t);
            // poses->push_back(Pose);
            // double timestamp = std::floor(std::stod(values[0]) * 1e5) * 1e-5;
            timeStamps->push_back(std::stod(values[0]));
        }       

    }

    int FindTimestampIdx(const double a, const std::vector<double> b)
    {
        double MinVal = DBL_MAX;
        int MinIdx = -1;

        for(int i = 0; i < b.size(); i++){
            double diff = std::fabs(b[i] - a);
            if(diff < MinVal){
                MinVal = diff;
                MinIdx = i;
            }
        }
        return MinIdx;
    }
}    