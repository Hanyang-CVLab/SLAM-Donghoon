#include "data.h"
// #include "common.h"

namespace dhSLAM{
    
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

                Vector6d pose = To6DOF(q, t); 
                poses->push_back(pose);
                timeStamps->push_back(std::stod(values[0]));
            }     
    }
}    