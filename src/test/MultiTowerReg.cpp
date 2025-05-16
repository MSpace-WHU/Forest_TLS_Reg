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
    std::string data_path = PROJECT_PATH;
    std::cout << BOLDGREEN << "----------------DATA PROCESSING----------------" << RESET << std::endl;
    std::cout << BOLDGREEN << "--------------------Reading Parameters-------------------" << RESET << std::endl;
    // read the setting parameters
    ConfigSetting config_setting;
    ReadParas(data_path+"/config/tower_para.yaml", config_setting);

    // variables for data
    std::vector<FrameInfo> frameInfoVec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_source(new pcl::PointCloud<pcl::PointXYZ>);
    
    std::vector<CandidateInfo> candidates_vec;
    std::vector<TLSPos> tlsVec;
    for(int i=1; i<=std::stoi(argv[1]); i++)
    {
        GTINDescManager *GTIN_map = new GTINDescManager(config_setting);
        ICP_target->clear();
        std::cout << BOLDGREEN << "--------------------Reading Target Data-------------------" << to_string(i) << RESET << std::endl;
        readTLSData(data_path+"/data/tower/"+ to_string(i) +".las", ICP_target);
        std::cout << "Read Target (" + std::to_string(i) +"), Points NUM: " << ICP_target->size() << std::endl;
        
        std::cout << BOLDGREEN << "----------------Generate Descriptor----------------" << RESET << std::endl;
        FrameInfo currMap;
        GTIN_map->GenTriDescs(ICP_target, currMap);
        GTIN_map->AddTriDescs(currMap);
        std::cout << "FrameID: " << i
            << " triangles NUM: " << currMap.desc_.size() 
            << " feature points: "<< currMap.currCenter->points.size() << std::endl;  
        
        CandidateInfo currt_candidate;
        currt_candidate.currFrameID = i-1;
        for(int j=1; j<=std::stoi(argv[1]); j++)
        {
            if(j != i)
            {
                ICP_source->clear();
                std::cout << BOLDGREEN << "--------------------Reading Source Data-------------------" << to_string(j) << RESET << std::endl;
                readTLSData(data_path+"/data/tower/"+ to_string(j) +".las", ICP_source);
                FrameInfo searchMap;
                GTIN_map->GenTriDescs(ICP_source, searchMap);
                
                std::pair<int, double> search_result(-1, 0);
                std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
                loop_transform.first << 0, 0, 0;
                loop_transform.second = Eigen::Matrix3d::Identity();
                std::vector<std::pair<TriDesc, TriDesc>> loop_triangle_pair;
                GTIN_map->SearchPosition(searchMap, search_result, loop_transform, loop_triangle_pair);
                
                if(search_result.first != -1 && search_result.second > config_setting.icp_threshold)
                {
                    int candID = j-1;
                    search_result.first = candID;
                    std::cout << "candidate ID: " << search_result.first << " score: " << search_result.second << std::endl;
                    
                    auto t_update_gicp_begin = std::chrono::high_resolution_clock::now();
                    trans_point_cloud(loop_transform, ICP_source);
                    // writeLas(data_path+"/data/tower/"+ std::to_string(candID+1)+"-rough_trans.las", ICP_source);
                    
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> refine_transform_gicp;
                    small_gicp_registration(ICP_source, ICP_target, refine_transform_gicp);
                    auto t_update_gicp_end = std::chrono::high_resolution_clock::now();
                    std::cout << "[Time] GICP Reg: " << time_inc(t_update_gicp_end, t_update_gicp_begin) << "ms" << std::endl;  
            
                    // trans
                    // trans_point_cloud(refine_transform_gicp, ICP_source);
                    // writeLas(data_path+"/data/tower/"+ std::to_string(candID+1)+"-fine_trans.las", ICP_source);
                    
                    // get the ICP fine registration
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> fine_trans;
                    fine_trans = matrixMultip(refine_transform_gicp, loop_transform);
                            
                    currt_candidate.candidateIDScore.push_back(search_result);
                    currt_candidate.relativePose.push_back(fine_trans);
                    currt_candidate.triMatch.push_back(loop_triangle_pair);
                }
            }
        }
        
        std::cout << BOLDGREEN << "currt_candidate.relativePose.size(): " << currt_candidate.relativePose.size() << RESET << std::endl;
        candidates_vec.push_back(currt_candidate);
        
        TLSPos tmpPos;
        tmpPos.ID = i;
        tmpPos.isValued = false;
        tlsVec.push_back(tmpPos);
    }

    int nodeID = -1, max_count = 0;
    std::vector<TLSPos> tlsVecFinal;
    // RelaToAbs(candidates_vec, tlsVec);
    for(int num=0; num<candidates_vec.size(); num++)
	{
		// calculated the initial pos of each TLS station
		for(int i=0; i<candidates_vec.size(); i++)
		{
			TLSPos currPos;
			int candtlsID, tlsID = candidates_vec[i].currFrameID;
			// set the first station
			if(tlsID == num)
			{
				currPos.ID = tlsID;
				currPos.R = Eigen::Matrix3d::Identity();
				currPos.t = Eigen::Vector3d::Zero();
				currPos.isValued = true;
				tlsVec[tlsID] = currPos;
			}
			// loop the candiates of each station, and calculate the pose
			for(int j=0; j<candidates_vec[i].candidateIDScore.size(); j++)
			{
				candtlsID = candidates_vec[i].candidateIDScore[j].first;
				// from currt to candidiate pose
				if(tlsVec[tlsID].isValued && !tlsVec[candtlsID].isValued)
				{
					currPos.ID = candtlsID;
					currPos.R = tlsVec[tlsID].R * candidates_vec[i].relativePose[j].second.inverse();
					currPos.t = tlsVec[tlsID].t - 
								tlsVec[tlsID].R * candidates_vec[i].relativePose[j].second.inverse() * candidates_vec[i].relativePose[j].first;
					currPos.isValued = true;
					tlsVec[candtlsID] = currPos;
				}
				// from candidiate to currt
				if(!tlsVec[tlsID].isValued && tlsVec[candtlsID].isValued)
				{
					currPos.ID = tlsID;
					currPos.R = tlsVec[candtlsID].R * candidates_vec[i].relativePose[j].second;
					currPos.t = tlsVec[candtlsID].R * candidates_vec[i].relativePose[j].first + tlsVec[candtlsID].t;
					currPos.isValued = true;
					tlsVec[tlsID] = currPos;
				}
			}
		}

        // stastic the number of seted nodessss
        int count = 0;
        for(int i=0; i<tlsVec.size(); i++)
        {
            if(tlsVec[i].isValued)
                count++;
        }

        // find the max number
        if(max_count < count)
        {
            // copy the value of tlsvec to tlsVecFinal, record max count of tlsvec
            max_count = count;
            nodeID = num;
            std::cout << "nodeID: " << nodeID << ", max_count: " << max_count << std::endl;
            tlsVecFinal.clear();
            tlsVecFinal = tlsVec;
        }

        // restore the tlsvec to default
        for(int i=0; i<tlsVec.size(); i++)
        {
            TLSPos tmpPos;
            tmpPos.ID = i;
            tmpPos.isValued = false;
            tlsVec[i] = tmpPos;
        }
	}

    write_pose(data_path+"/data/tower/Poses.txt", tlsVecFinal);

    pcl::PointCloud<pcl::PointXYZ>::Ptr total_las(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<tlsVecFinal.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr single_scan(new pcl::PointCloud<pcl::PointXYZ>);
        if(tlsVecFinal[i].isValued)
        {
            // std::cout <<"tlsVecFinal: \n" << tlsVecFinal[i].t << "\n" << tlsVecFinal[i].R << std::endl;

            readTLSData(data_path+"/data/tower/"+ to_string(tlsVecFinal[i].ID+1) +".las", single_scan);
            std::cout << "Read LAS (" + std::to_string(tlsVecFinal[i].ID+1) +"), Points NUM: " << single_scan->size() << std::endl;
            
            // down sample
            down_sampling_voxel(*single_scan, 0.05);

            std::pair<Eigen::Vector3d, Eigen::Matrix3d> trans, transINV;
            trans.first = tlsVecFinal[i].t;
            trans.second = tlsVecFinal[i].R;
            transINV = matrixInv(trans);
            trans_point_cloud(transINV, single_scan);
            
            *total_las += *single_scan;
        }
    }

    // sor filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(total_las);
    sor.setMeanK(50); // neighborhood size
    sor.setStddevMulThresh(1.0); // thres of standard error
    // applied to filter
    sor.filter(*total_las);

    writeLas(data_path+"/data/tower/total.las", total_las);

    std::cout<< BOLDGREEN << "Finish!" << RESET << std::endl;
}