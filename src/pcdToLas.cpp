#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "Hlp.h"
#endif

// C++
#include <iostream>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace std;
using namespace pcl;
using namespace Eigen;

int main(int argc, char **argv) 
{
    std::string data_path = "/media/xiaochen/xch_disk/subplot";
    for(int i=0; i<=624; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(data_path+"/"+std::to_string(i)+".pcd", *cloud);

        writeLas(data_path+"/"+std::to_string(i)+".las", cloud);
    }
    return 0;
}