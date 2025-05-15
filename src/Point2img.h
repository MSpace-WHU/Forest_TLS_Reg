
#ifndef _DST_H_Included_
#define _DST_H_Included_
#include "DST.h"
#endif

#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "Hlp.h"
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include<opencv2/core/eigen.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>

void point2img(pcl::PointCloud<pcl::PointXYZ>::Ptr ori_pc,
               int Horizon_SCAN, int N_SCAN, 
               std::vector<std::vector<POINT_D>> &grid_data,
               cv::Mat &pointMat);

void img2point(std::vector<std::vector<POINT_D>> &grid_data,
               cv::Mat &pointMat,
               pcl::PointCloud<pcl::PointXYZ>::Ptr out_pc);

// for ground
void horiGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr ori_pc,
               int grid_num, double range, 
               std::vector<std::vector<POINT_D>> &grid_data);