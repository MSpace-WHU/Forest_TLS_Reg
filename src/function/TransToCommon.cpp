#include "math.h"

#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "../include/utils/Hlp.h"
#endif

// C++
#include <iostream>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;


std::vector<Eigen::Affine3d> readStationData(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return {};
    }

    std::string line;
    std::vector<Eigen::Affine3d> transforms;

    while (std::getline(file, line)) {
        if (line.find("Station:") != std::string::npos) {
            Eigen::Affine3d transform = Eigen::Affine3d::Identity();

            // Read the translation vector (with comma separation)
            if (!std::getline(file, line)) break; // Check if the next line is valid
            line.erase(remove(line.begin(), line.end(), ' '), line.end());
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream translationStream(line);
            Eigen::Vector3d translation;
            if (!(translationStream >> translation[0] >> translation[1] >> translation[2])) {
                std::cerr << "Error reading translation vector." << std::endl;
                continue; // Skip to the next station
            }
            transform.translation() = translation;

            // Read the rotation matrix (3x3)
            Eigen::Matrix3d rotation;
            for (int i = 0; i < 3; ++i) {
                if (!std::getline(file, line)) break; // Check if the next line is valid
                std::istringstream rotationStream(line);
                for (int j = 0; j < 3; ++j) {
                    if (!(rotationStream >> rotation(i, j))) {
                        std::cerr << "Error reading rotation matrix." << std::endl;
                        break; // Handle error
                    }
                }
            }
            transform.linear() = rotation;

            // Skip the last line (the 1 0 0 ... line)
            std::getline(file, line);

            // Store the transformation in the vector
            transforms.push_back(transform);
        }
    }

    return transforms;
}

int main(int argc, char **argv) 
{
    // change the data path
    // /media/xiaochen/xch_disk/RegTLSData/Whu-TLS/4-Forest/0-TLS-forest-benchmark-DWX/
    // /media/xiaochen/xch_disk/RegTLSData/ETH-Trees/
    std::string data_path = "/media/xiaochen/xch_disk/RegTLSData/Tongji-Trees/Plot4/";
    // std::cout << "datapath: " << data_path+"/finalPoses.txt"<< std::endl;
    
    // change the data path of pose file
    std::vector<Eigen::Affine3d> tlsTrans = readStationData(data_path+"findMinum-0-huber-kernal-score0.4/icpPoses.txt");
    
    std::cout << "tlsTrans.size(): "<< tlsTrans.size() << std::endl;
    std::ofstream outfile(data_path+"/transed/scan_positions.txt");
    for(int i=0; i<tlsTrans.size(); i++)
    {
        // std::cout << "Trans-" << i << ": " << tlsTrans[i].translation().transpose() << std::endl;
        // Eigen::Affine3d transINV = tlsTrans[i].inverse();
        pcl::PointCloud<pcl::PointXYZ>::Ptr las_data(new pcl::PointCloud<pcl::PointXYZ>);
        // readTLSData(data_path + to_string(i+1) + ".las", las_data);
        // pcl::io::loadPLYFile(data_path+ "s" + std::to_string(i+1)+".ply", *las_data);
        pcl::io::loadPLYFile(data_path+ "S" + std::to_string(i+1)+".ply", *las_data);

        std::cout << "reference_data-" << to_string(i+1) << ": " << las_data->size() << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr out_data(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*las_data, *out_data, tlsTrans[i].matrix());
        
        std::string las_file = data_path + "/transed/" + std::to_string(i+1) + "-transed.las";
        writeLas(las_file, out_data);

        outfile << tlsTrans[i].translation().transpose() << std::endl;
    }
    outfile.close();
}
