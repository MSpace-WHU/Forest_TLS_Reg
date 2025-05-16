#include "math.h"

#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "../include/utils/Hlp.h"
#endif

// C++
#include <iostream>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;

int main(int argc, char **argv) 
{
    std::string data_path = PROJECT_PATH;
    std::vector<Eigen::Affine3d> tlsTrans = readTLSTrans(data_path+"/data/snj/transformation.txt");
    std::cout << "tlsTrans.size(): "<< tlsTrans.size() << std::endl;
    for(int i=0; i<tlsTrans.size(); i++)
    {
        Eigen::Affine3d transINV = tlsTrans[i].inverse();
        pcl::PointCloud<pcl::PointXYZ>::Ptr las_data(new pcl::PointCloud<pcl::PointXYZ>);
        readTLSData(data_path+"/data/snj/" + argv[1] + "-" +to_string(i+1)+".las", las_data);
        std::cout << "target_data-" << to_string(i+1) << ": " << las_data->size() << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr out_data(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*las_data, *out_data, transINV.matrix());
        
        std::string las_file = data_path + "/data/snj/" + std::to_string(i+1) + ".las";
        writeLas(las_file, out_data);
    }
}
