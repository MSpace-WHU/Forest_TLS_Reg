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

#ifndef _FEC_H_Included_
#define _FEC_H_Included_
#include "../include/utils/FEC.h"
#endif

#ifndef _POINT2IMG_H_Included_
#define _POINT2IMG_H_Included_
#include "../include/utils/Point2img.h"
#endif

#include <mutex>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include "CSF.h"

#include <fast_gicp/gicp/fast_gicp.hpp>
// #include "pointmatcher/PointMatcher.h"
#include <small_gicp/registration/registration_helper.hpp>

// sort the node by vote number
bool sortByVoteNumber(const std::pair<int, int> a, const std::pair<int, int> b);
// time increment
double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin);

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec);
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi);

// CSF, set the parameters, and get the indices of ground and object
void clothSimulationFilter(const vector< csf::Point >& pc,
                           vector<int> &groundIndexes,
                           vector<int> & offGroundIndexes,
                           ConfigSetting &config_setting);

// spereate the ground points
void addPointCloud(const vector<int>& index_vec, 
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

// FEC cluster, get the cluster points
void fec_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points_,
                 ConfigSetting &config_setting);

void clusters_joined(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &obj_cluster_points,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obj);

// calculate the attributes of each cluster
void cluster_attributes(std::vector<Cluster> &clusters,
                        std::vector<Cluster> &disgard_clusters,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points_,
                        ConfigSetting &config_setting);

// 
void ground_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_data, 
                 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &ground,
                 ConfigSetting &config_setting);
void gnd_cluster_attributes(std::vector<Cluster> &clusters,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points_,
                        ConfigSetting &config_setting);

// get the total clusters
void get_total_clusters(std::vector<Cluster> &obj_clusters, 
                        std::vector<Cluster> &gnd_clusters,
                        std::vector<Cluster> &total_clusters,
                        ConfigSetting &config_setting);

void trans_point_cloud(std::pair<Eigen::Vector3d, Eigen::Matrix3d> res,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &current_cloud);

void trans_point_cloud(std::pair<Eigen::Vector3d, Eigen::Matrix3d> res,
                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr &current_cloud);

void TriMatchList2Points(const TriMatchList &candidate_matcher,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt);

void TriMatchList2Matrix(const TriMatchList &candidate_matcher,
                         Eigen::Matrix3Xd &cloud_src,
                         Eigen::Matrix3Xd &cloud_tgt);

// void fast_gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
//                             pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
//                             std::pair<Eigen::Vector3d, Eigen::Matrix3d> &refine_transform);

void small_gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
                            std::pair<Eigen::Vector3d, Eigen::Matrix3d> &refine_transform);

void pcl_gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
                            std::pair<Eigen::Vector3d, Eigen::Matrix3d> &refine_transform);
                            
class GTINDescManager
{
public:
    ConfigSetting config_setting_;
    std::vector<FrameInfo> frame_info_vec_;
    
    // hash table, save all descriptors
    std::unordered_map<TriDesc_LOC, std::vector<TriDesc>> data_base_;

    // current frame id
    unsigned int current_frame_id_;
    
    std::vector<Cluster> obj_clusters;
    std::vector<Cluster> discarded_clusters;
    
    // store the clusters
    std::vector<std::vector<Cluster>> clusters_vec_;
    std::vector<std::vector<Cluster>> discarded_clusters_vec;
    std::vector<cv::Mat> pointMat_vec_;
    std::vector<cv::Mat> pointTypeMat_vec_;
    
    // ground and off-ground points
    pcl::PointCloud<pcl::PointXYZ> ground;
    pcl::PointCloud<pcl::PointXYZ> offground;
public:

    // generate triangle descriptor
    void GenTriDescs(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
                                    FrameInfo &curr_frame_info);

    // const std::vector<TriDesc> &stds_vec 
    void AddTriDescs(const FrameInfo &curr_frame_info);

    // search the loop for current fram, "stds_vec"
    void SearchPosition(
        const FrameInfo &curr_frame, std::pair<int, double> &loop_result,
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
        std::vector<std::pair<TriDesc, TriDesc>> &loop_std_pair);
    
    void SearchMultiPosition(
        const FrameInfo &curr_frame, std::vector<std::pair<int, double>> &loop_result,
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &loop_transform,
        std::vector<std::vector<std::pair<TriDesc, TriDesc>>> &loop_std_pair_vec);
    
    // the current frame is used for searthing, select the candidate frame
    void candidate_frames_selector(const FrameInfo &curr_frame,
                                    std::vector<TriMatchList> &candidate_matcher_vec);

    // using svd to find the best pose, and verify whether it is valid match
    void candidate_frames_verify(const FrameInfo &curr_frame,
        const TriMatchList &candidate_matcher, double &verify_score,
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
        std::vector<std::pair<TriDesc, TriDesc>> &sucess_match_vec,
        ConfigSetting &config_setting); 
    
    void init_clusterTLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);
    void init_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);
    
    void find_cluster_root(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                        std::vector<Cluster> &clusters);

    void getPoint(FrameInfo &curr_frame_info);
    
    // calculated the inital pose by the triangle vetices, using SVD solver
    void triangle_solver(std::pair<TriDesc, TriDesc> &std_pair,
                        Eigen::Vector3d &t, Eigen::Matrix3d &rot); 

    // verfey the normal and distance, the ratio of correct match points to all source points
    double geometric_verify(
                const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
                const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
                std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);
    // build STDescs from corner points.
    void build_stdesc(FrameInfo &curr_frame_info);

    // construct function
    GTINDescManager(ConfigSetting &config_setting) : config_setting_(config_setting)
    {
        current_frame_id_ = 0;
    }


};

