#include "math.h"

#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "../include/utils/Hlp.h"
#endif

#ifndef _DST_H_Included_
#define _DST_H_Included_
#include "../include/dst/DST.h"
#endif

#ifndef _GENALG_H_Included_
#define _GENALG_H_Included_
#include "../include/utils/GenAlg.h"
#endif

#ifndef _POINT2IMG_H_Included_
#define _POINT2IMG_H_Included_
#include "../include/utils/Point2img.h"
#endif

#ifndef _FEC_H_Included_
#define _FEC_H_Included_
#include "../include/utils/FEC.h"
#endif

#ifndef _TriObj_H_Included_
#define _TriObj_H_Included_
#include "../include/utils/TriObj.h"
#endif

// C++
#include <iostream>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;

int main(int argc, char **argv) 
{
    std::string data_path = PROJECT_PATH;
    std::cout << BOLDGREEN << "----------------DATA PROCESSING----------------" << RESET << std::endl;
    std::cout << BOLDGREEN << "--------------------Reading Parameters-------------------" << RESET << std::endl;
    // read the setting parameters
    ConfigSetting config_setting;
    ReadParas(data_path+"/config/tower_para.yaml", config_setting);

    std::cout << BOLDGREEN << "--------------------Reading Las Data-------------------" << RESET << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_data(new pcl::PointCloud<pcl::PointXYZ>);
    readTLSData(data_path+"/data/tower/"+ argv[1]+".las", target_data);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_data(new pcl::PointCloud<pcl::PointXYZ>);
    readTLSData(data_path+"/data/tower/"+ argv[2]+".las", source_data);
    
    std::cout << "Read Target Points NUM: " << target_data->size() << std::endl;
    std::cout << "Read Source Points NUM: " << source_data->size() << std::endl;
    
    std::cout << BOLDGREEN << "----------------Generate Descriptor----------------" << RESET << std::endl;
    GTINDescManager *GTIN_map = new GTINDescManager(config_setting);
    // generate the tri descriptor and save (Target)
    FrameInfo currMap;
    GTIN_map->GenTriDescs(target_data, currMap);
    
    writeLas(data_path+"/data/tower/"+ argv[1]+"-obj.las", currMap.currPoints);
    pcl::io::savePCDFileASCII(data_path+"/data/tower/"+ argv[1]+"-obj_center.pcd", *currMap.currCenter);
    GTIN_map->AddTriDescs(currMap);
    
    // generate the tri descriptor (Source)
    FrameInfo searchMap;
    GTIN_map->GenTriDescs(source_data, searchMap);
    writeLas(data_path+"/data/tower/"+ argv[2]+"-obj.las", searchMap.currPoints);
    pcl::io::savePCDFileASCII(data_path+"/data/tower/"+ argv[2]+"-obj_center.pcd", *searchMap.currCenter);
   
    std::cout << BOLDGREEN << "----------------Searching----------------" << RESET << std::endl;
    auto t_query_begin = std::chrono::high_resolution_clock::now();
    std::pair<int, double> search_result(-1, 0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    loop_transform.first << 0, 0, 0;
    loop_transform.second = Eigen::Matrix3d::Identity();
    std::vector<std::pair<TriDesc, TriDesc>> loop_triangle_pair;
    GTIN_map->SearchPosition(searchMap, search_result, loop_transform, loop_triangle_pair);
    auto t_query_end = std::chrono::high_resolution_clock::now();
    std::cout << "[Time] query: " << time_inc(t_query_end, t_query_begin) << "ms" << std::endl;

    std::cout << "search_result: " << search_result.first << ", " << search_result.second << std::endl;
    std::cout << "loop_transform.first: \n" << loop_transform.first << std::endl;
    std::cout << "loop_transform.second: \n" << loop_transform.second << std::endl;

    std::cout << "FrameID: " << currMap.frame_id_
            << " triangles NUM: " << currMap.desc_.size() 
            << " feature points: "<< currMap.currCenter->points.size() << std::endl;

    // find the corresponding TLS
    if(search_result.first != -1)
    {
        // trans the source data, and save (rough trans data)
        // down_sampling_voxel(*source_data, 0.03);
        trans_point_cloud(loop_transform, source_data);
        writeLas(data_path+"/data/tower/"+ argv[2]+"-rough_trans.las", source_data);
        // trans obj points, and save (rough trans data)
        trans_point_cloud(loop_transform, searchMap.currPoints);
        // writeLas(data_path+"/data/tower/"+ argv[2]+"-obj-transed.las", searchMap.currPoints);
        
        // down_sampling_voxel(*target_data, 0.03);
        std::cout << BOLDGREEN << "----------------Fine Registrating----------------" << RESET << std::endl;
        // fine registration by GICP
        auto t_update_gicp_begin = std::chrono::high_resolution_clock::now();
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> refine_transform_gicp;
        // std::cout << "Here is good!" << std::endl;
        // fast_gicp_registration(source_data, target_data, refine_transform_gicp);
        small_gicp_registration(source_data, target_data, refine_transform_gicp);
        // pcl_gicp_registration(source_data, target_data, refine_transform_gicp);
        auto t_update_gicp_end = std::chrono::high_resolution_clock::now();
        std::cout << "[Time] FastGICP: " << time_inc(t_update_gicp_end, t_update_gicp_begin) << "ms" << std::endl;  

        // finely trans the source data, and save
        trans_point_cloud(refine_transform_gicp, source_data);
        writeLas(data_path+"/data/tower/"+ argv[2]+"-fine_trans.las", source_data);

        // get the fine registration
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> fine_trans;
        fine_trans.second = refine_transform_gicp.second * loop_transform.second;
        fine_trans.first = refine_transform_gicp.second * loop_transform.first + refine_transform_gicp.first;
        write_relative_pose(data_path+"/data/tower/"+ argv[1] + "-" + argv[2]+"_fine_pose.txt", fine_trans);
        std::cout << "fine_trans.first: \n" << fine_trans.first << std::endl;
        std::cout << "fine_trans.second: \n" << fine_trans.second << std::endl;
    }

    std::cout << BOLDGREEN << "----------------Finish!----------------" << RESET << std::endl;
    return 0;   
}