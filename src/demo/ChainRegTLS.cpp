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

#ifndef _HashRegObj_H_Included_
#define _HashRegObj_H_Included_
#include "../include/utils/HashRegObj.h"
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
    // std::string data_path = "/home/xiaochen/workspace/Forest_TLS_Reg_ws";
    std::cout << BOLDGREEN << "----------------DATA PROCESSING----------------" << RESET << std::endl;
    // read the setting parameters
    ConfigSetting config_setting;
    ReadParas(data_path+"/config/snj_para.yaml", config_setting);    
    // read the grund truth of pose data
    std::vector<Eigen::Affine3d> tlsTrans = readTLSTrans(data_path+"/data/snj/transformation.txt");
    std::cout << "tlsTrans size(): "<< tlsTrans.size() << std::endl;
    
    // read TLS data
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_data(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_data(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> fine_pose;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rough_error, fine_error;
    for(int i=1; i<tlsTrans.size(); i++)
    {
        std::cout << BOLDGREEN << "--------------------Reading TLS Data-------------------" << RESET << std::endl;
        Eigen::Affine3d gt_pose = tlsTrans[i-1].inverse() * tlsTrans[i];
        reference_data->clear();
        source_data->clear();
        readTLSData(data_path+"/data/snj/"+ std::to_string(i) +".las", reference_data);    
        readTLSData(data_path+"/data/snj/"+ std::to_string(i+1) +".las", source_data);
        std::cout << "Read Target (" + std::to_string(i) +"), Points NUM: " << reference_data->size() << std::endl;
        std::cout << "Read Source (" + std::to_string(i+1) +"), Points NUM: " << source_data->size() << std::endl;

        std::cout << BOLDGREEN << "----------------Generate Descriptor----------------" << RESET << std::endl;
        HashRegDescManager *HashReg_RefTLS = new HashRegDescManager(config_setting);
        // generate the tri descriptor and save (Target)
        FrameInfo reference_tls_info;
        HashReg_RefTLS->GenTriDescs(reference_data, reference_tls_info);
        writeLas(data_path+"/data/snj/"+ std::to_string(i)+"-obj.las", reference_tls_info.currPoints);
        pcl::io::savePCDFileASCII(data_path+"/data/snj/"+ std::to_string(i) +"-obj_center.pcd", *reference_tls_info.currCenter);
        HashReg_RefTLS->AddTriDescs(reference_tls_info);
        // generate the tri descriptor (Source)
        FrameInfo searchMap;
        HashReg_RefTLS->GenTriDescs(source_data, searchMap);
        writeLas(data_path+"/data/snj/"+ std::to_string(i+1) +"-obj.las", searchMap.currPoints);
        pcl::io::savePCDFileASCII(data_path+"/data/snj/"+ std::to_string(i+1) +"-obj_center.pcd", *searchMap.currCenter);

        std::cout << BOLDGREEN << "----------------Searching----------------" << RESET << std::endl;
        auto t_query_begin = std::chrono::high_resolution_clock::now();
        std::pair<int, double> search_result(-1, 0);
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
        loop_transform.first << 0, 0, 0;
        loop_transform.second = Eigen::Matrix3d::Identity();
        std::vector<std::pair<TriDesc, TriDesc>> loop_triangle_pair;
        HashReg_RefTLS->SearchPosition(searchMap, search_result, loop_transform, loop_triangle_pair);
        auto t_query_end = std::chrono::high_resolution_clock::now();
        
        std::cout << "[Time] query: " << time_inc(t_query_end, t_query_begin) << "ms" << std::endl;
        std::cout << "search_result: " << search_result.first << ", " << search_result.second << std::endl;
        std::cout << "loop_transform.first: \n" << loop_transform.first << std::endl;
        std::cout << "loop_transform.second: \n" << loop_transform.second << std::endl;

        std::cout << "FrameID: " << reference_tls_info.frame_id_
                << " triangles NUM: " << reference_tls_info.desc_.size() 
                << " feature points: "<< reference_tls_info.currCenter->points.size() << std::endl;    
        
        if(search_result.first == -1)
        {
            std::cout << BOLDRED << "Failed Registration!!" << std::endl;
            continue;
        }
        if(search_result.first != -1)
        {            
            // down sample the target and source point cloud
            // down_sampling_voxel(*source_data, 0.015);
            // down_sampling_voxel(*reference_data, 0.03);

            // accurcy evaluation
            std::pair<Eigen::Vector3d, Eigen::Vector3d> rough_tmp;
            accur_evaluation(loop_transform, gt_pose, rough_tmp);
            // write_error(data_path+"/data/snj/"+ std::to_string(i+1) +"-rough-error.txt", rough_tmp);
            rough_error.push_back(rough_tmp);

            // trans the source data to target (rough transform)
            trans_point_cloud(loop_transform, source_data);

            std::cout << BOLDGREEN << "----------------Fine Registrating----------------" << RESET << std::endl;
            std::cout << BOLDGREEN << std::to_string(i) << "-" << std::to_string(i+1) << RESET << std::endl;
            auto t_update_gicp_begin = std::chrono::high_resolution_clock::now();
            std::pair<Eigen::Vector3d, Eigen::Matrix3d> refine_transform_gicp;
            // fast_gicp_registration(source_data, reference_data, refine_transform_gicp);
            small_gicp_registration(source_data, reference_data, refine_transform_gicp);
            auto t_update_gicp_end = std::chrono::high_resolution_clock::now();
            std::cout << "[Time] Small_GICP: " << time_inc(t_update_gicp_end, t_update_gicp_begin) << "ms" << std::endl;  

            // finely trans the source data, and save
            trans_point_cloud(refine_transform_gicp, source_data);
            writeLas(data_path+"/data/snj/"+ std::to_string(i+1) +"-fine_trans.las", source_data);

            // get the fine registration
            std::pair<Eigen::Vector3d, Eigen::Matrix3d> fine_trans;
            fine_trans.first = refine_transform_gicp.second * loop_transform.first + refine_transform_gicp.first;
            fine_trans.second = refine_transform_gicp.second * loop_transform.second;
            fine_pose.push_back(fine_trans);

            // accurcy evaluation (fine)
            std::pair<Eigen::Vector3d, Eigen::Vector3d> fine_tmp;
            accur_evaluation(fine_trans, gt_pose, fine_tmp);
            // write_error(data_path+"/data/snj/"+ std::to_string(i+1) +"-fine-error.txt", fine_tmp);
            fine_error.push_back(fine_tmp);
        } 
    }

    std::cout << BOLDGREEN << "----------------Stastic the POSE and ERROR----------------" << RESET << std::endl;
    std::cout << "Fine_pose NUM: " << fine_pose.size() << std::endl;
    std::ofstream outPoseFile(data_path+"/data/snj/pose.txt");
    std::ofstream outRoughErrFile(data_path+"/data/snj/roughErr.txt");
    std::ofstream outFineErrFile(data_path+"/data/snj/fineErr.txt");

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> curr_pose;
    curr_pose.first = Eigen::Vector3d::Zero(); curr_pose.second = Eigen::Matrix3d::Identity();
    outPoseFile << "Station " << std::to_string(1) << std::endl;
    outPoseFile << curr_pose.first.transpose() << std::endl;
    outPoseFile << curr_pose.second << "\n" << std::endl;

    for(int i=0; i<fine_pose.size(); i++)
    {
        if (!outPoseFile.is_open() || !outRoughErrFile.is_open() || !outFineErrFile.is_open()) 
        {
            std::cerr << "failed open this file!" << std::endl;
        }
        
        curr_pose.first = curr_pose.second*fine_pose[i].first + curr_pose.first;
        curr_pose.second = curr_pose.second * fine_pose[i].second;
        outPoseFile << "Station " << std::to_string(i+2) << std::endl;
        outPoseFile << curr_pose.first.transpose() << std::endl;
        outPoseFile << curr_pose.second << "\n" << std::endl;
        
        outRoughErrFile << "Station " << std::to_string(i+2) << std::endl;
        outRoughErrFile << rough_error[i].first.transpose();
        outRoughErrFile << " ";
        outRoughErrFile << rough_error[i].second.transpose() << "\n" << std::endl;
        
        outFineErrFile << "Station " << std::to_string(i+2) << std::endl;
        outFineErrFile << fine_error[i].first.transpose();
        outFineErrFile << " ";
        outFineErrFile << fine_error[i].second.transpose() << "\n" << std::endl;
    }
    outPoseFile.close();
    outRoughErrFile.close();
    outFineErrFile.close();

    std::pair<Eigen::Vector3d, Eigen::Vector3d> total_error;
    Eigen::Affine3d total_gt = tlsTrans[0].inverse() * tlsTrans[fine_pose.size()];
    accur_evaluation(curr_pose, total_gt, total_error);
    write_error(data_path+"/data/snj/total-error.txt", total_error);
    std::cout << BOLDGREEN << "----------------Finish!----------------" << RESET << std::endl;
    return 0;
}
