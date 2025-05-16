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
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;

int main(int argc, char **argv) 
{
    std::string data_path = "/home/xiaochen/workspace/Forest_TLS_Reg_ws";
    std::cout << BOLDGREEN << "----------------DATA PROCESSING----------------" << RESET << std::endl;
    std::cout << BOLDGREEN << "--------------------Reading Parameters-------------------" << RESET << std::endl;
    // read the setting parameters
    ConfigSetting config_setting;
    ReadParas(data_path+"/config/tower_para.yaml", config_setting);

    // variables for data
    std::vector<FrameInfo> frameInfoVec;

    std::vector<CandidateInfo> candidates_vec;
    for(int i=1; i<std::stoi(argv[1]); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_target(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_source(new pcl::PointCloud<pcl::PointXYZ>);
    
        GTINDescManager *GTIN_map = new GTINDescManager(config_setting);
        std::cout << BOLDGREEN << "--------------------Reading Target Data-------------------" << to_string(i) << RESET << std::endl;
        readTLSData(data_path+"/data/tower/"+ to_string(i) +".las", ICP_target);
        readTLSData(data_path+"/data/tower/"+ to_string(i+1) +".las", ICP_source);
        std::cout << "Read Target (" + std::to_string(i) +"), Points NUM: " << ICP_target->size() << std::endl;
        std::cout << "Read Source (" + std::to_string(i+1) +"), Points NUM: " << ICP_source->size() << std::endl;

        std::cout << BOLDGREEN << "----------------Generate Target Descriptor----------------" << RESET << std::endl;
        FrameInfo currMap;
        GTIN_map->GenTriDescs(ICP_target, currMap);
        GTIN_map->AddTriDescs(currMap);
        std::cout << "FrameID: " << i
            << " triangles NUM: " << currMap.desc_.size() 
            << " feature points: "<< currMap.currCenter->points.size() << std::endl;  
        std::cout << BOLDGREEN << "----------------Generate Source Descriptor----------------" << RESET << std::endl;
        FrameInfo searchMap;
        GTIN_map->GenTriDescs(ICP_source, searchMap);
        
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
        
        CandidateInfo currt_candidate;
        currt_candidate.currFrameID = i-1;
        
        if(search_result.first == -1)
        {
            std::cout << BOLDRED << "Failed Registration!!" << std::endl;
            
            currt_candidate.candidateIDScore.push_back(search_result);
            currt_candidate.relativePose.push_back(loop_transform);
            currt_candidate.triMatch.push_back(loop_triangle_pair);
            candidates_vec.push_back(currt_candidate);

            continue;
        }

        if(search_result.first != -1)
        {
            std::cout << BOLDGREEN << "----------------Fine Registrating----------------" << RESET << std::endl;
            std::cout << BOLDGREEN << std::to_string(i) << "-" << std::to_string(i+1) << RESET << std::endl;
            
            trans_point_cloud(loop_transform, ICP_source);
            auto t_update_gicp_begin = std::chrono::high_resolution_clock::now();
            std::pair<Eigen::Vector3d, Eigen::Matrix3d> refine_transform_gicp;
            // fast_gicp_registration(source_data, target_data, refine_transform_gicp);
            small_gicp_registration(ICP_source, ICP_target, refine_transform_gicp);
            auto t_update_gicp_end = std::chrono::high_resolution_clock::now();
            std::cout << "[Time] FastGICP: " << time_inc(t_update_gicp_end, t_update_gicp_begin) << "ms" << std::endl;  
            
            trans_point_cloud(refine_transform_gicp, ICP_source);
            // writeLas(data_path+"/data/tower/"+ std::to_string(i+1) +"-fine_trans.las", ICP_source);
            
            // get the fine registration
            std::pair<Eigen::Vector3d, Eigen::Matrix3d> fine_trans;
            fine_trans.first = refine_transform_gicp.second * loop_transform.first + refine_transform_gicp.first;
            fine_trans.second = refine_transform_gicp.second * loop_transform.second;
            write_relative_pose(data_path+"/data/tower/"+std::to_string(i) + "-" + std::to_string(i+1) +"_fine_pose.txt", fine_trans);
            
            currt_candidate.candidateIDScore.push_back(search_result);
            currt_candidate.relativePose.push_back(fine_trans);
            currt_candidate.triMatch.push_back(loop_triangle_pair);
            candidates_vec.push_back(currt_candidate);
        }  
    }

    // get the start and end indexes from a series candidate poses
    int count = 0; // count the number of continuous possitive elements
    int maxCount = 0; // max number
    int startIndex = -1; // start index of the elements
    int endIndex = -1; // end index
    int tempStart = -1; // temp variable of the start index
    for (int i = 0; i < candidates_vec.size(); ++i) {
        if (candidates_vec[i].candidateIDScore[0].first != -1) {
            count++; // increase, if the ID is not -1
            if (tempStart == -1) {
                tempStart = i; // record the temp start index
            }
        } else {
            // if encounter -1, check the count and the max count
            if (count > maxCount) { 
                maxCount = count; // update the max count
                startIndex = tempStart; // update the start index
                endIndex = i - 1; // update the end index
            }
            count = 0; // reset
            tempStart = -1; // reset
        }
    }
    // check the last segment
    if (count > maxCount) {
        maxCount = count;
        startIndex = tempStart; // update the start index
        endIndex = candidates_vec.size() - 1; // update the end index
    }
    std::cout<< BOLDGREEN << "startIndex: " << startIndex << ", endIndex: " << endIndex << RESET << std::endl;
    
    // get the pose of each node
    std::vector<TLSPos> tlsVec;
    TLSPos tmpPos;
    tmpPos.ID = candidates_vec[startIndex].currFrameID;
    tmpPos.R = Eigen::Matrix3d::Identity();
    tmpPos.t = Eigen::Vector3d::Zero();
    tmpPos.isValued = true;
    tlsVec.push_back(tmpPos);
    for(int i=startIndex; i<=endIndex; i++)
    {
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> trans;
        if(tlsVec[i-startIndex].isValued)
        {
            std::pair<Eigen::Vector3d, Eigen::Matrix3d> curr_trans, relative_trans;
            
            curr_trans.first = tlsVec[i-startIndex].t;
            curr_trans.second = tlsVec[i-startIndex].R;
            relative_trans = candidates_vec[i].relativePose[0];

            trans = matrixMultip(curr_trans, relative_trans);
        }

        TLSPos currPos;
        currPos.ID = candidates_vec[i].currFrameID+1;
        currPos.R = trans.second;
        currPos.t = trans.first;
        currPos.isValued = true;

        tlsVec.push_back(currPos);
    }
    write_pose(data_path+"/data/tower/Poses.txt", tlsVec);

    // trans the TLS data into a common coordinate
    pcl::PointCloud<pcl::PointXYZ>::Ptr total_las(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<tlsVec.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr single_scan(new pcl::PointCloud<pcl::PointXYZ>);
        if(tlsVec[i].isValued)
        {
            // std::cout <<"tlsVec: \n" << tlsVec[i].t << "\n" << tlsVec[i].R << std::endl;

            readTLSData(data_path+"/data/tower/"+ to_string(tlsVec[i].ID+1) +".las", single_scan);
            std::cout << "Read LAS (" + std::to_string(tlsVec[i].ID+1) +"), Points NUM: " << single_scan->size() << std::endl;
            
            // down sample
            down_sampling_voxel(*single_scan, 0.05);

            std::pair<Eigen::Vector3d, Eigen::Matrix3d> trans, transINV;
            trans.first = tlsVec[i].t;
            trans.second = tlsVec[i].R;
            transINV = matrixInv(trans);
            trans_point_cloud(trans, single_scan);
            
            *total_las += *single_scan;
        }
    }

    // sor filter
    auto t_sor_filter_begin = std::chrono::high_resolution_clock::now();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(total_las);
    sor.setMeanK(50); // neighborhood size
    sor.setStddevMulThresh(1.0); // thres of standard error
    // applied to filter
    sor.filter(*total_las);

    writeLas(data_path+"/data/tower/total.las", total_las);
    auto t_sor_filter_end = std::chrono::high_resolution_clock::now();
    std::cout << "[Time] SOR Filter: " << time_inc(t_sor_filter_end, t_sor_filter_begin) << "ms" << std::endl;  
            
    std::cout<< BOLDGREEN << "Finish!" << RESET << std::endl;
}