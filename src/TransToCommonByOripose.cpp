#include "math.h"

#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "Hlp.h"
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


int main(int argc, char **argv) 
{
    // change the data path
    // /media/xiaochen/xch_disk/RegTLSData/Whu-TLS/4-Forest/0-TLS-forest-benchmark-DWX/
    // /media/xiaochen/xch_disk/RegTLSData/ETH-Trees/
    std::string data_path = "/media/xiaochen/xch_disk/RegTLSData/SNJ_Data/snj-040/downsampled/";

    std::vector<Eigen::Affine3d> tlsTrans;
    tlsTrans = readTLSTrans(data_path+"transformation.txt");

    std::cout << "tlsTrans.size(): "<< tlsTrans.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_data(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<tlsTrans.size(); i++)
    {
        // Eigen::Affine3d transINV = tlsTrans[i].inverse();
        pcl::PointCloud<pcl::PointXYZ>::Ptr las_data(new pcl::PointCloud<pcl::PointXYZ>);
        // readTLSData(data_path + to_string(i+1) + ".las", las_data);
        // pcl::io::loadPLYFile(data_path+ "s" + std::to_string(i+1)+".ply", *las_data);
        pcl::io::loadPLYFile(data_path+ std::to_string(i+1)+".ply", *las_data);

        std::cout << "Trans-" << i+1 << ": " << las_data->points.size() << " points!" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transed_data(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*las_data, *transed_data, tlsTrans[i].matrix());
        *out_data += *transed_data;
    }
    // std::string las_file = data_path + "/transed/" + std::to_string(i+1) + "-transed.las";
    // writeLas(las_file, out_data);
    std::cout << BOLDGREEN << "Writting the total points cloud!" << RESET << std::endl;
    std::string pcd_file = data_path + "total.pcd";
    pcl::io::savePCDFile(pcd_file, *out_data);
    std::cout << BOLDGREEN << "Finished!" << RESET << std::endl;
}
