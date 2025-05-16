
#ifndef _POINT2IMG_H_Included_
#define _POINT2IMG_H_Included_
#include "../include/utils/Point2img.h"
#endif

void point2img(pcl::PointCloud<pcl::PointXYZ>::Ptr ori_pc,
               int Horizon_SCAN, int N_SCAN, 
               std::vector<std::vector<POINT_D>> &grid_data,
               cv::Mat &pointMat)
{
    double horiRes = 360.0 / Horizon_SCAN;
    double vertiRes = 150.0 / N_SCAN;
    int NUM = ori_pc->size();
    // loop all point cloud
    for (int i=0; i<NUM; i++)
    {
        pcl::PointXYZ point = ori_pc->points[i];
        
        double d = sqrt(point.x * point.x + point.y * point.y);
        double azimuth = (atan2(point.y, point.x) * 180.0f / M_PI) + 180;
        double elevation = (atan2(point.z, d) * 180.0f / M_PI);
        
        int indxX, indxY;
        if(azimuth>=0 && azimuth < 360)
            indxX = int(azimuth/horiRes);
        else
            continue;
        if(elevation>=-60 && elevation<90)
            indxY = int((elevation+60)/vertiRes);
        else
            continue;

        if(grid_data[indxX][indxY].d <0 || d < grid_data[indxX][indxY].d)
        {
            grid_data[indxX][indxY].p = point;
            grid_data[indxX][indxY].d = d;
        }
    }
    
    // to cv image
    for(int i=0; i<Horizon_SCAN; i++)
    {
        for(int j=0; j<N_SCAN; j++)
        {
            if(grid_data[i][j].d >0)
                pointMat.at<float>(i, j) = grid_data[i][j].d;
        }
    }  
}

void img2point(std::vector<std::vector<POINT_D>> &grid_data,
               cv::Mat &pointMat,
               pcl::PointCloud<pcl::PointXYZ>::Ptr out_pc)
{
    for(int i=0; i<pointMat.rows; i++)
    {
        for(int j=0; j<pointMat.cols; j++)
        {
            if(pointMat.at<float>(i, j) == 1)
                out_pc->push_back(grid_data[i][j].p);
        }
    }
}

// for ground
void horiGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr ori_pc,
               int grid_num, double range, 
               std::vector<std::vector<POINT_D>> &grid_data,
               cv::Mat &pointMat)
{
    double grid_size = range/grid_num;
    int NUM = ori_pc->size();
    // loop all point cloud
    for (int i=0; i<NUM; i++)
    {
        pcl::PointXYZ point = ori_pc->points[i];
        if(abs(point.x)<=range && abs(point.y)<=range)
        {
            int indxX, indxY;
            indxX = int(point.x / grid_size)+grid_num;
            indxY = int(point.y / grid_size)+grid_num;

            if(point.z < grid_data[indxX][indxY].p.z)
            {
                grid_data[indxX][indxY].p = point;
                grid_data[indxX][indxY].d = point.z;
            }
        }
    }

}
