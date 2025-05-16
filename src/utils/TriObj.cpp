#ifndef _TriObj_H_Included_
#define _TriObj_H_Included_
#include "../include/utils/TriObj.h"
#endif
#include <omp.h>


// sort the node by vote number
bool sortByVoteNumber(const std::pair<int, int> a, const std::pair<int, int> b)
{
	return a.second > b.second;
};

// time difference between two time step
double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin) 
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count() * 1000;
}

// trans between vector and the point
pcl::PointXYZI vec2point(const Eigen::Vector3d &vec) {
  pcl::PointXYZI pi;
  pi.x = vec[0];
  pi.y = vec[1];
  pi.z = vec[2];
  return pi;
}
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi) {
  return Eigen::Vector3d(pi.x, pi.y, pi.z);
}

// CSF, set the parameters, and get the indices of ground and object
void clothSimulationFilter(const vector< csf::Point >& pc,
                           vector<int> &groundIndexes,
                           vector<int> & offGroundIndexes,
                           ConfigSetting &config_setting)
{
	//step 1 read point cloud
    CSF csf;
	csf.setPointCloud(pc);// or csf.readPointsFromFile(pointClouds_filepath); 
	
    //step 2 parameter settings
	csf.params.bSloopSmooth = config_setting.bSloopSmooth;
	csf.params.cloth_resolution = config_setting.cloth_resolution;
	csf.params.rigidness = config_setting.rigidness;

	csf.params.time_step = config_setting.time_step;
	csf.params.class_threshold = config_setting.class_threshold;
	csf.params.interations = config_setting.interations;

	//step 3 do filtering
	csf.do_filtering(groundIndexes, offGroundIndexes);
}

// spereate the ground points, selected the points by index
void addPointCloud(const vector<int>& index_vec, 
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	auto& points = cloud_filtered->points;
	const auto& pointclouds = cloud->points;

	for_each(index_vec.begin(), index_vec.end(), [&](const auto& index) {
		pcl::PointXYZ pc;
		pc.x = pointclouds[index].x;
		pc.y = pointclouds[index].y;
		pc.z = pointclouds[index].z;
		// pc.intensity = pointclouds[index].intensity;

		points.push_back(pc);
	});

	cloud_filtered->height = 1;
	cloud_filtered->width = cloud_filtered->points.size();
}

// FEC cluster, get the cluster points
void fec_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points_,
                 ConfigSetting &config_setting)
{
    if(cloud->size() <= 0)
        return;
    int min_component_size = config_setting.min_component_size;
    double tolorance = config_setting.tolorance;
    int max_n = config_setting.max_n;
    std::vector<pcl::PointIndices> cluster_indices;
    // fec, get the indices of clusters
    cluster_indices = FEC(cloud, min_component_size, tolorance, max_n);
    
    // loop all clusters
    pcl::PointCloud<pcl::PointXYZ>::Ptr centers_(new pcl::PointCloud<pcl::PointXYZ>);
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_points_;
    for (int i = 0; i < cluster_indices.size(); i++) 
    {
        pcl::PointXYZ center_tmp_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr points_tmp_(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < cluster_indices[i].indices.size(); j++)
        {
            center_tmp_.x += cloud->points[cluster_indices[i].indices[j]].x;
            center_tmp_.y += cloud->points[cluster_indices[i].indices[j]].y;
            center_tmp_.z += cloud->points[cluster_indices[i].indices[j]].z;
            
            points_tmp_->push_back(cloud->points[cluster_indices[i].indices[j]]);
        }
        center_tmp_.x /= cluster_indices[i].indices.size();
        center_tmp_.y /= cluster_indices[i].indices.size();
        center_tmp_.z /= cluster_indices[i].indices.size();

        centers_->push_back(center_tmp_);
        cluster_points_.push_back(points_tmp_);
    }
    
    for (size_t i = 0; i < centers_->size(); i++)
    {
        for (size_t j = i+1; j < centers_->size();)
        {
            double dx = centers_->points[i].x - centers_->points[j].x;
            double dy = centers_->points[i].y - centers_->points[j].y;
            double dz = centers_->points[i].z - centers_->points[j].z;
            double d2 = sqrt(dx*dx + dy*dy);
            double d3 = sqrt(dx*dx + dy*dy + dz*dz);
            
            if(d2 < config_setting.merge_dist)
            {
                *cluster_points_[i] = *cluster_points_[i] + *cluster_points_[j];
                
                centers_->points[i].x = (centers_->points[i].x + centers_->points[j].x) / 2;
                centers_->points[i].y = (centers_->points[i].y + centers_->points[j].y) / 2;
                centers_->points[i].z = (centers_->points[i].z + centers_->points[j].z) / 2;

                centers_->erase(centers_->begin() + j);
                cluster_points_.erase(cluster_points_.begin() + j);
            }
            else
            {
                j++;
            }
        }
    }
    
    // std::cout << "cluster_indices size: " << cluster_indices.size() 
    //           << ", cluster_points_: " << cluster_points_.size() << std::endl;
}

void clusters_joined(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &obj_cluster_points,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obj)
{
    if(obj_cluster_points.size() <= 0)
        return;
    
    for(int i=0; i<obj_cluster_points.size(); i++)
    {
        float randomFloat = rand()%255; //distribution(generator);
        for(int j=0; j<obj_cluster_points[i]->size(); j++)
        {
            pcl::PointXYZI p;
            p.x = obj_cluster_points[i]->points[j].x;
            p.y = obj_cluster_points[i]->points[j].y;
            p.z = obj_cluster_points[i]->points[j].z;

            p.intensity = randomFloat;
            cloud_obj->push_back(p);
        }
    }
    
}

// calculate the attributes of each cluster, then select the trunk
void cluster_attributes(std::vector<Cluster> &clusters,
                        std::vector<Cluster> &disgard_clusters,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points_,
                        ConfigSetting &config_setting)
{
    if(cluster_points_.size() <= 0)
        return;

    // center of obj cloud
    Cluster cluster_tmp;
    
    // int line=0, plane=0;
    
    // loop all clusters
    // #pragma omp parallel for num_threads(8)
    for (int i = 0; i < cluster_points_.size(); i++) 
    {
        // init value
        cluster_tmp.center_ = Eigen::Vector3d::Zero();
        cluster_tmp.covariance_ = Eigen::Matrix3d::Zero();
        cluster_tmp.normal_ = Eigen::Vector3d::Zero();
        cluster_tmp.eig_value_ = Eigen::Vector3d::Zero();
        cluster_tmp.points_.clear();

        double minZ = MAX_INF;
        double maxZ = MIN_INF;
        // loop the points in each cluster
        for (int j = 0; j < cluster_points_[i]->points.size(); j++)
        {
            Eigen::Vector3d pi;
            pi[0] = cluster_points_[i]->points[j].x;
            pi[1] = cluster_points_[i]->points[j].y;
            pi[2] = cluster_points_[i]->points[j].z;
            if (minZ > pi[2])
                minZ = pi[2];
            if (maxZ < pi[2])
                maxZ = pi[2];
            cluster_tmp.center_ += pi;
            cluster_tmp.covariance_ += pi * pi.transpose();
        }
        cluster_tmp.points_ = *cluster_points_[i];

        // calculate the center and covariance
        cluster_tmp.center_ = cluster_tmp.center_ / cluster_points_[i]->points.size();
        cluster_tmp.covariance_ = cluster_tmp.covariance_/cluster_points_[i]->points.size() -
                                cluster_tmp.center_ * cluster_tmp.center_.transpose();
        cluster_tmp.center_[2] = minZ; // minZ; 1.5
        cluster_tmp.minZ = minZ;
        cluster_tmp.maxZ = maxZ;

        Eigen::EigenSolver<Eigen::Matrix3d> es(cluster_tmp.covariance_);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3d::Index evalsMin, evalsMax; 
        evalsReal.minCoeff(&evalsMin);
        evalsReal.maxCoeff(&evalsMax); 
        int evalsMid = 3 - evalsMin - evalsMax;
        // the attributes in cluster
        cluster_tmp.linearity_ = (evalsReal(evalsMax) - evalsReal(evalsMid)) / evalsReal(evalsMax);
        cluster_tmp.planarity_ = (evalsReal(evalsMid) - evalsReal(evalsMin)) / evalsReal(evalsMax);
        cluster_tmp.scatering_ = evalsReal(evalsMin) / evalsReal(evalsMax);
        cluster_tmp.eig_value_ << evalsReal(evalsMax), evalsReal(evalsMid), evalsReal(evalsMin);
        cluster_tmp.normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
                        evecs.real()(2, evalsMax);
        Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d normal_inc = cluster_tmp.normal_ - Z;
        Eigen::Vector3d normal_add = cluster_tmp.normal_ + Z; 
        // std::cout << "normal_inc: " << normal_inc.norm() << ", normal_add: " << normal_add.norm() << std::endl;
        // put it in the cluster
        if(cluster_tmp.scatering_ > config_setting.scateringThres)
        {
            // center point and the direction of line (trunk)
            cluster_tmp.p_center_.x = cluster_tmp.center_[0];
            cluster_tmp.p_center_.y = cluster_tmp.center_[1];
            cluster_tmp.p_center_.z = cluster_tmp.center_[2];
            cluster_tmp.p_center_.normal_x = cluster_tmp.normal_[0];
            cluster_tmp.p_center_.normal_y = cluster_tmp.normal_[1];
            cluster_tmp.p_center_.normal_z = cluster_tmp.normal_[2];
            cluster_tmp.p_center_.intensity = 1;
                        
            cluster_tmp.is_line_ = false;
            disgard_clusters.push_back(cluster_tmp);
            continue;
        }
        else if(cluster_tmp.linearity_ > config_setting.linearityThres && 
                (normal_inc.norm()< config_setting.upThres || normal_add.norm()<config_setting.upThres) &&
                (cluster_tmp.maxZ - cluster_tmp.minZ) > config_setting.clusterHeight)
        {
            // center point and the direction of line (trunk)
            cluster_tmp.p_center_.x = cluster_tmp.center_[0];
            cluster_tmp.p_center_.y = cluster_tmp.center_[1];
            cluster_tmp.p_center_.z = cluster_tmp.center_[2];
            cluster_tmp.p_center_.normal_x = cluster_tmp.normal_[0];
            cluster_tmp.p_center_.normal_y = cluster_tmp.normal_[1];
            cluster_tmp.p_center_.normal_z = cluster_tmp.normal_[2];
            cluster_tmp.p_center_.intensity = 1;
                        
            cluster_tmp.is_line_ = true;
            clusters.push_back(cluster_tmp);
            // line++;
        }
        else
        {
            // center point and the direction of line (trunk)
            cluster_tmp.p_center_.x = cluster_tmp.center_[0];
            cluster_tmp.p_center_.y = cluster_tmp.center_[1];
            cluster_tmp.p_center_.z = cluster_tmp.center_[2];
            cluster_tmp.p_center_.normal_x = cluster_tmp.normal_[0];
            cluster_tmp.p_center_.normal_y = cluster_tmp.normal_[1];
            cluster_tmp.p_center_.normal_z = cluster_tmp.normal_[2];
            cluster_tmp.p_center_.intensity = 1;
                        
            cluster_tmp.is_line_ = false;
            disgard_clusters.push_back(cluster_tmp);
        }
    }
    // std::cout << "Line: " << line << ", Plane: " << plane << std::endl;
}

// init the ground grids, sperate the points into each grid
void ground_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_data, 
                 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &ground,
                 ConfigSetting &config_setting)
{
    if(ground_data->size() <= 0)
        return;
    double grid_size = config_setting.gnd_grid_size;
    double points_num = config_setting.gnd_points_num;
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> ground_grids;
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*ground_data, min_pt, max_pt);
    // enough size
    int x_size = ceil(abs(max_pt.x - min_pt.x)/grid_size)+1;
    int y_size = ceil(abs(max_pt.y - min_pt.y)/grid_size)+1;
    // std::cout << "----x_size: " << x_size << ", y_size: " << y_size << std::endl;
    ground_grids.resize(x_size);
    for (int i = 0; i < x_size; ++i) {
        ground_grids[i].resize(y_size);
        for(int j=0; j< y_size; j++)
        {
            ground_grids[i][j] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }
    }
    
    // split in 2d grid
    for(int i = 0; i < ground_data->points.size(); ++i)
    {
        int ind_x = floor(abs(ground_data->points[i].x - min_pt.x)/grid_size);
        int ind_y = floor(abs(ground_data->points[i].y - min_pt.y)/grid_size);
        ground_grids[ind_x][ind_y]->push_back(ground_data->points[i]);
    }
    
    // reconstruct in one dimsion
    for(int i=0; i<x_size; i++)
    {
        for(int j=0; j<y_size; j++)
        {
            if(ground_grids[i][j]->points.size() > points_num)
                ground.push_back(ground_grids[i][j]);
        }
    }
}

// calculate the attributes of ground clusters
void gnd_cluster_attributes(std::vector<Cluster> &clusters,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_points_,
                        ConfigSetting &config_setting)
{
    if(cluster_points_.size() <= 0)
        return;
    // center of obj cloud
    Cluster cluster_tmp;
    for (int i = 0; i < cluster_points_.size(); i++) 
    {
        // init value
        cluster_tmp.center_ = Eigen::Vector3d::Zero();
        cluster_tmp.covariance_ = Eigen::Matrix3d::Zero();
        cluster_tmp.normal_ = Eigen::Vector3d::Zero();
        cluster_tmp.eig_value_ = Eigen::Vector3d::Zero();
        cluster_tmp.points_.clear();
        // loop the points in each cluster
        double min_z = cluster_points_[i]->points[0].z;
        for (int j = 0; j < cluster_points_[i]->points.size(); j++)
        {
            Eigen::Vector3d pi;
            pi[0] = cluster_points_[i]->points[j].x;
            pi[1] = cluster_points_[i]->points[j].y;
            pi[2] = cluster_points_[i]->points[j].z;
            
            cluster_tmp.center_ += pi;
            cluster_tmp.covariance_ += pi * pi.transpose();

            if(min_z > cluster_points_[i]->points[j].z)
                min_z = cluster_points_[i]->points[j].z;
        }
        cluster_tmp.points_ = *cluster_points_[i];

        // calculate the center and covariance
        cluster_tmp.center_ = cluster_tmp.center_ / cluster_points_[i]->points.size();
        cluster_tmp.covariance_ = cluster_tmp.covariance_/cluster_points_[i]->points.size() -
                                cluster_tmp.center_ * cluster_tmp.center_.transpose();
        Eigen::EigenSolver<Eigen::Matrix3d> es(cluster_tmp.covariance_);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3d::Index evalsMin, evalsMax; 
        evalsReal.minCoeff(&evalsMin);
        evalsReal.maxCoeff(&evalsMax); 
        int evalsMid = 3 - evalsMin - evalsMax;
        // the attributes in cluster
        cluster_tmp.linearity_ = (evalsReal(evalsMax) - evalsReal(evalsMid)) / evalsReal(evalsMax);
        cluster_tmp.planarity_ = (evalsReal(evalsMid) - evalsReal(evalsMin)) / evalsReal(evalsMax);
        cluster_tmp.scatering_ = evalsReal(evalsMin) / evalsReal(evalsMax);
        cluster_tmp.eig_value_ << evalsReal(evalsMax), evalsReal(evalsMid), evalsReal(evalsMin);

        // if(cluster_tmp.scatering_ > config_setting.scateringThres)
        //     continue;
        // else if (cluster_tmp.planarity_ > config_setting.planarityThres)
        // {
            // center point and the normal of plane
            cluster_tmp.normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
                                    evecs.real()(2, evalsMin);
            cluster_tmp.p_center_.x = cluster_tmp.center_[0];
            cluster_tmp.p_center_.y = cluster_tmp.center_[1];
            cluster_tmp.p_center_.z = cluster_tmp.center_[2];
            // cluster_tmp.p_center_.z = min_z;
            cluster_tmp.p_center_.normal_x = cluster_tmp.normal_[0];
            cluster_tmp.p_center_.normal_y = cluster_tmp.normal_[1];
            cluster_tmp.p_center_.normal_z = cluster_tmp.normal_[2];
            cluster_tmp.p_center_.intensity = 0;
            
            cluster_tmp.is_plane_ = true;
            clusters.push_back(cluster_tmp);
            // plane++;
        // }
    }
}

// get the total clusters
void get_total_clusters(std::vector<Cluster> &obj_clusters, 
                        std::vector<Cluster> &gnd_clusters,
                        std::vector<Cluster> &total_clusters,
                        ConfigSetting &config_setting)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr obj_center_points(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr gnd_center_points(new pcl::PointCloud<pcl::PointXYZINormal>);
    // wether the clusters is zero
    if(obj_clusters.size() == 0 && gnd_clusters.size() == 0)
    {
        std::cout << "obj_clusters.size(): " << obj_clusters.size() 
                  << ", gnd_clusters.size(): " <<  gnd_clusters.size() << std::endl;
        
        std::cout << "The clustes NUM is 0, so return." << std::endl;
        return;
    }
    
    // get the object cluster center points
    for(auto iter=obj_clusters.begin(); iter!=obj_clusters.end(); iter++)
    {
        obj_center_points->push_back(iter->p_center_);
        total_clusters.push_back(*iter);
    }

    if(gnd_clusters.size() == 0)
        return;
    
    // get the ground cluster center points
    for(auto iter=gnd_clusters.begin(); iter!=gnd_clusters.end(); iter++)
        gnd_center_points->push_back(iter->p_center_);
    
    // Find the nearest points from ground (using KD tree)
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kd_tree(
                            new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
    std::vector<int> pointIdxNKNSearch; // near indexs
    std::vector<float> pointNKNSquaredDistance; // near distance
    kd_tree->setInputCloud(gnd_center_points);
    int nearestNUM = 3;
    for(int i=0; i<obj_center_points->points.size(); i++)
    {
        pcl::PointXYZINormal searchPoint = obj_center_points->points[i];
        kd_tree->nearestKSearch(searchPoint, nearestNUM, pointIdxNKNSearch, pointNKNSquaredDistance);
        for(int j=0; j<pointIdxNKNSearch.size(); j++) //nearestNUM
        {
            pcl::PointXYZINormal targetPoint = gnd_center_points->points[pointIdxNKNSearch[j]];
            double dx = searchPoint.x - targetPoint.x;
            double dy = searchPoint.y - targetPoint.y;
            double dxy = sqrt(dx*dx + dy*dy);
            // is a plane, and the distance in xy plane is smaller than 1 meter
            if(gnd_center_points->points[pointIdxNKNSearch[j]].intensity==0 &&
                dxy<config_setting.gnd_points_dist)
            {
                total_clusters.push_back(gnd_clusters[pointIdxNKNSearch[j]]);
                break;
            }
        }
    }
}

// trans the point cloud data
void trans_point_cloud(std::pair<Eigen::Vector3d, Eigen::Matrix3d> res,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &current_cloud)
{
  Eigen::Vector3d translation = res.first;
  Eigen::Matrix3d rotation = res.second;

  // transform the curr cloud coordinate
  for(int j=0; j<current_cloud->points.size(); j++)
  {
    pcl::PointXYZ point;
    point = current_cloud->points[j];

    Eigen::Vector3d pv;
    pv << point.x, point.y, point.z;
    pv = rotation * pv + translation;
    
    point.x = pv[0];
    point.y = pv[1];
    point.z = pv[2];
    current_cloud->points[j] = point;
  }
  // set the property
  current_cloud->width = current_cloud->size();
  current_cloud->height = 1;      
}

// trans the point cloud data
void trans_point_cloud(std::pair<Eigen::Vector3d, Eigen::Matrix3d> res,
                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr &current_cloud)
{
  Eigen::Vector3d translation = res.first;
  Eigen::Matrix3d rotation = res.second;

  // transform the curr cloud coordinate
  for(int j=0; j<current_cloud->points.size(); j++)
  {
    pcl::PointXYZINormal point;
    point = current_cloud->points[j];

    Eigen::Vector3d pv;
    pv << point.x, point.y, point.z;
    pv = rotation * pv + translation;
    
    point.x = pv[0];
    point.y = pv[1];
    point.z = pv[2];
    current_cloud->points[j] = point;
  }  
  current_cloud->width = current_cloud->size();
  current_cloud->height = 1;     
}

// trans the center points of the matched triangle into points or matrix
void TriMatchList2Points(const TriMatchList &candidate_matcher,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt)
{
    for(int i=0; i<candidate_matcher.match_list_.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = candidate_matcher.match_list_[i].first.center_[0];
        p.y = candidate_matcher.match_list_[i].first.center_[1];
        p.z = candidate_matcher.match_list_[i].first.center_[2];
        cloud_src->push_back(p);
        p.x = candidate_matcher.match_list_[i].second.center_[0];
        p.y = candidate_matcher.match_list_[i].second.center_[1];
        p.z = candidate_matcher.match_list_[i].second.center_[2];
        cloud_tgt->push_back(p);
    }
}

void TriMatchList2Matrix(const TriMatchList &candidate_matcher,
                         Eigen::Matrix3Xd &cloud_src,
                         Eigen::Matrix3Xd &cloud_tgt)
{
    int num = candidate_matcher.match_list_.size();
    cloud_src.resize(3, num);
    cloud_tgt.resize(3, num);

    for(int i=0; i<candidate_matcher.match_list_.size(); i++)
    {
        pcl::PointXYZ p;
        cloud_src(0,i) = candidate_matcher.match_list_[i].first.center_[0];
        cloud_src(1,i) = candidate_matcher.match_list_[i].first.center_[1];
        cloud_src(2,i) = candidate_matcher.match_list_[i].first.center_[2];
        
        cloud_tgt(0,i) = candidate_matcher.match_list_[i].second.center_[0];
        cloud_tgt(1,i) = candidate_matcher.match_list_[i].second.center_[1];
        cloud_tgt(2,i) = candidate_matcher.match_list_[i].second.center_[2];
    }
}

// void fast_gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
//                             pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
//                             std::pair<Eigen::Vector3d, Eigen::Matrix3d> &refine_transform)
// {
//     // auto gicp = boost::make_shared<fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>>();
//     // gicp->setNumThreads(4);
//     // gicp->setInputSource(source);
//     // gicp->setInputTarget(target);
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//     // gicp->align(*aligned_temp_cloud);
//     // // get the transformation data
//     // Eigen::Matrix4f trans = gicp->getFinalTransformation();

//     fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
//     gicp.setNumThreads(4);
//     gicp.setInputSource(source);
//     gicp.setInputTarget(target);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//     gicp.align(*aligned_temp_cloud);
//     // get the transformation data
//     Eigen::Matrix4f trans = gicp.getFinalTransformation();

//     matrix_to_pair(trans, refine_transform);
// }

void small_gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
                            std::pair<Eigen::Vector3d, Eigen::Matrix3d> &refine_transform)
{
    std::vector<Eigen::Vector3d> target_points;   // Any of Eigen::Vector(3|4)(f|d) can be used
    std::vector<Eigen::Vector3d> source_points;   // 
    
    point_to_vector(source, source_points);
    point_to_vector(target, target_points);
    
    small_gicp::RegistrationSetting setting;
    setting.num_threads = 4;                    // Number of threads to be used
    setting.downsampling_resolution = 0.05;     // Downsampling resolution
    setting.max_correspondence_distance = 0.8;  // Maximum correspondence distance between points (e.g., triming threshold)
    // std::cout << "hello-1" << std::endl;
    Eigen::Isometry3d init_T_target_source = Eigen::Isometry3d::Identity();
    small_gicp::RegistrationResult result = small_gicp::align(target_points, source_points, init_T_target_source, setting);
    // std::cout << "hello-2" << std::endl;
    size_t num_inliers = result.num_inliers;       // Number of inlier source points
    Eigen::Matrix<double, 6, 6> H = result.H;      // Final Hessian matrix (6x6)
    
    // get the transformation
    Eigen::Isometry3d T = result.T_target_source;  // Estimated transformation
    Eigen::Isometry3f Tf = T.cast<float>();
    Eigen::Matrix4f trans = Tf.matrix();
    matrix_to_pair(trans, refine_transform);
}

void pcl_gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
                            std::pair<Eigen::Vector3d, Eigen::Matrix3d> &refine_transform)
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    icp.setMaxCorrespondenceDistance(0.8);
    icp.setMaximumIterations (20);
    icp.setTransformationEpsilon (1e-8);
    icp.setEuclideanFitnessEpsilon (1);
    
    icp.setInputSource(source);
    icp.setInputTarget(target);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "hasConverged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    
    Eigen::Matrix4f trans = icp.getFinalTransformation();
    matrix_to_pair(trans, refine_transform);
}

/********************THREE FUNCTIONS**********************/
// generate triangle descriptor
void GTINDescManager::GenTriDescs(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
                                    FrameInfo &curr_frame_info)
{
    curr_frame_info.currCenter.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    curr_frame_info.currCenterFix.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    curr_frame_info.currPoints.reset(new pcl::PointCloud<pcl::PointXYZ>);
    init_cluster(input_cloud);
    // init_clusterTLS(input_cloud);
    // std::cout << BOLDRED << "obj_clusters[2] : " << obj_clusters[2].center_[2] << ", p_center_.z: " << obj_clusters[2].p_center_.z << RESET << std::endl;

    getPoint(curr_frame_info);
    // std::cout << "curr_frame_info.currCenter->points.size(): " 
    //         << curr_frame_info.currCenter->points.size() << std::endl;
    
    // build the descriptor
    curr_frame_info.desc_.clear();
    if(curr_frame_info.currCenter->points.size() > 0)
        build_stdesc(curr_frame_info);

    // // get current frame id
    curr_frame_info.frame_id_ = current_frame_id_;
    current_frame_id_++;
    
    clusters_vec_.push_back(obj_clusters);
    discarded_clusters_vec.push_back(discarded_clusters);

    // release the memeroy
    obj_clusters.clear();
    discarded_clusters.clear();
}

// add triangle descriptor to data base   const std::vector<TriDesc> &trids_vec
void GTINDescManager::AddTriDescs(const FrameInfo &curr_frame_info) {
  // get the tri descriptor  
  std::vector<TriDesc> trids_vec;
  trids_vec = curr_frame_info.desc_;
  // update frame id
  for (auto single_std : trids_vec) {
    // calculate the position of single std
    TriDesc_LOC position;
    // ceil
    position.x = (int)(single_std.side_length_[0] + 0.5);
    position.y = (int)(single_std.side_length_[1] + 0.5);
    position.z = (int)(single_std.side_length_[2] + 0.5);
    position.a = (int)(single_std.angle_[0]);
    position.b = (int)(single_std.angle_[1]);
    position.c = (int)(single_std.angle_[2]);
    auto iter = data_base_.find(position);
    if (iter != data_base_.end()) {
      data_base_[position].push_back(single_std);
    } else {
      std::vector<TriDesc> descriptor_vec;
      descriptor_vec.push_back(single_std);
      data_base_[position] = descriptor_vec;
    }
  }
  // push the frames info to vector  
  frame_info_vec_.push_back(curr_frame_info);
  return;
}

// search the loop for current fram, "stds_vec"
void GTINDescManager::SearchPosition(
    const FrameInfo &curr_frame, std::pair<int, double> &loop_result,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
    std::vector<std::pair<TriDesc, TriDesc>> &loop_std_pair) 
{
    // no data
    if (curr_frame.desc_.size() == 0) {
        std::cout << BOLDRED << "No Triangle Descs!" << RESET << std::endl;
        loop_result = std::pair<int, double>(-1, 0);
        return;
    }
    
    // step1, select candidates, default number 50
    auto t1 = std::chrono::high_resolution_clock::now();    
    std::vector<TriMatchList> candidate_matcher_vec;
    candidate_frames_selector(curr_frame, candidate_matcher_vec);
    auto t2 = std::chrono::high_resolution_clock::now();

    // DEBUG 
    std::cout << "candidate_matcher_vec.size(): " << candidate_matcher_vec.size() << std::endl;
    std::cout << "curr_frame: " << curr_frame.desc_.size() << std::endl;
    if(candidate_matcher_vec.size()>0)
    {
       std::cout << "first matcher list size: " <<candidate_matcher_vec[0].match_list_.size()
        << ", last matcher list size: " << candidate_matcher_vec.back().match_list_.size() << std::endl;
    }

    // step2, select best candidates from rough candidates
    double best_score = 0;
    unsigned int best_candidate_id = -1;
    unsigned int triggle_candidate = -1;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> best_transform;
    std::vector<std::pair<TriDesc, TriDesc>> best_sucess_match_vec;
    for (size_t i = 0; i < candidate_matcher_vec.size(); i++) 
    {
        double verify_score = -1;
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> relative_pose;
        std::vector<std::pair<TriDesc, TriDesc>> sucess_match_vec;
        candidate_frames_verify(curr_frame, candidate_matcher_vec[i], verify_score, 
                                relative_pose, sucess_match_vec, config_setting_);

        if (verify_score > best_score)
        {
            best_score = verify_score;
            best_candidate_id = candidate_matcher_vec[i].match_id_.second;
            best_transform = relative_pose;
            best_sucess_match_vec = sucess_match_vec;
            triggle_candidate = i;
        }
    }
    auto t3 = std::chrono::high_resolution_clock::now();

    std::cout << "[Time] candidate selector: " << time_inc(t2, t1)
                << " ms, candidate verify: " << time_inc(t3, t2) << "ms"
                << std::endl;
    std::cout << "best_score: " << best_score << std::endl;
    
    if (best_score >= config_setting_.icp_threshold) 
    {
        std::cout << "best_candidate_id: " << best_candidate_id 
                    << ", best_score: " << best_score << std::endl;
        loop_result = std::pair<int, double>(best_candidate_id, best_score);
        loop_transform = best_transform;
        loop_std_pair = best_sucess_match_vec;
        return;
    } 
    else 
    { // the loop closure is failed
        loop_result = std::pair<int, double>(-1, 0);
        return;
    }
}

// search the loop for current fram, "stds_vec"
void GTINDescManager::SearchMultiPosition(
    const FrameInfo &curr_frame, std::vector<std::pair<int, double>> &loop_result,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &loop_transform,
    std::vector<std::vector<std::pair<TriDesc, TriDesc>>> &loop_std_pair_vec) 
{
    std::pair<int, double> id_score;
    // no data
    if (curr_frame.desc_.size() == 0) {
        std::cout << BOLDRED << "No Triangle Descs!" << RESET << std::endl;
        id_score = std::pair<int, double>(-1, 0);
        loop_result.push_back(id_score);
        return;
    }
    
    // step1, select candidates, default number 50
    auto t1 = std::chrono::high_resolution_clock::now();    
    std::vector<TriMatchList> candidate_matcher_vec;
    candidate_frames_selector(curr_frame, candidate_matcher_vec);
    auto t2 = std::chrono::high_resolution_clock::now();

    // DEBUG 
    std::cout << "candidate_matcher_vec.size(): " << candidate_matcher_vec.size() << std::endl;
    std::cout << "curr_frame: " << curr_frame.desc_.size() << std::endl;
    if(candidate_matcher_vec.size()>0)
    {
       std::cout << "first matcher list size: " <<candidate_matcher_vec[0].match_list_.size()
        << ", last matcher list size: " << candidate_matcher_vec.back().match_list_.size() << std::endl;
    }

    // step2, select candidates by geomatic verify
    for (size_t i = 0; i < candidate_matcher_vec.size(); i++) 
    {
        double verify_score = -1;
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> relative_pose;
        std::vector<std::pair<TriDesc, TriDesc>> sucess_match_vec;
        
        candidate_frames_verify(curr_frame, candidate_matcher_vec[i], verify_score, 
                                relative_pose, sucess_match_vec, config_setting_);

        if (verify_score > 0)
        {
            id_score.first = candidate_matcher_vec[i].match_id_.second;
            id_score.second = verify_score;
            loop_result.push_back(id_score);
            loop_transform.push_back(relative_pose);
            loop_std_pair_vec.push_back(sucess_match_vec);
        }   
    }
    
    // step3, sort the candidates by score
    for(int i=loop_transform.size()-1; i>0; --i)
    {
        for (int j = 0; j < i; ++j)
        {
            if(loop_result[j].second < loop_result[j+1].second)
            {
                swap(loop_result[j], loop_result[j+1]);
                swap(loop_transform[j], loop_transform[j+1]);
                swap(loop_std_pair_vec[j], loop_std_pair_vec[j+1]);
            }
        }
    }
    auto t3 = std::chrono::high_resolution_clock::now();

    std::cout << "[Time] candidate selector: " << time_inc(t2, t1)
                << " ms, candidate verify: " << time_inc(t3, t2) << "ms"
                << std::endl;
    
    // // step4, select the best ID and score
    // double best_score = loop_result[0].second;
    // unsigned int best_candidate_id = loop_result[0].first;
    // std::pair<Eigen::Vector3d, Eigen::Matrix3d> best_transform = loop_transform[0];
    // std::vector<std::pair<TriDesc, TriDesc>> best_sucess_match_vec = loop_std_pair_vec[0];
    // std::cout << "best_score: " << best_score << std::endl;
    
    // if (best_score >= config_setting_.icp_threshold) 
    // {
    //     std::cout << "best_candidate_id: " << best_candidate_id 
    //                 << ", best_score: " << best_score << std::endl;
    //     return;
    // } 
    // else 
    // { // the loop closure is failed
    //     return;
    // }
}

/********************THREE FUNCTIONS**********************/

/********************USEFUL FUNCTIONS**********************/

// init the cluster
void GTINDescManager::init_clusterTLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obj(new pcl::PointCloud<pcl::PointXYZ>);
    
    unsigned long size, i=0, j=0;
    
    size = input_cloud->size();
    // std::cout << "Total PointCloud has: " << size << " points." << std::endl;
   
    // trans to CSF point
    vector<csf::Point> pc;
    const auto& pointclouds = input_cloud->points;
    pc.resize(input_cloud->size());
    transform(pointclouds.begin(), pointclouds.end(), pc.begin(), [&](const auto& p)->csf::Point {
        csf::Point pp;
        pp.x = p.x;
        pp.y = p.y;
        pp.z = p.z;
        return pp;
    });
    // std::cout << "here is good-1: " << pc.size() << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();
    // get the index of ground and offground points 
    std::vector<int> groundIndexes, offGroundIndexes;
    clothSimulationFilter(pc, groundIndexes, offGroundIndexes, config_setting_);
    // std::cout << "here is good-2" << std::endl;
    addPointCloud(groundIndexes, input_cloud, cloud_ground);
    addPointCloud(offGroundIndexes, input_cloud, cloud_obj);
    auto t2 = std::chrono::high_resolution_clock::now();
    pc.clear();
    ground = *cloud_ground;
    offground = *cloud_obj;
    std::cout << "cloud_ground: " << cloud_ground->size()
                << ", cloud_obj: " << cloud_obj->size() << std::endl;
    
    // cloud_obj to img
    std::vector<std::vector<POINT_D>> grid_data(config_setting_.imgRow, std::vector<POINT_D>(config_setting_.imgCol));
    cv::Mat pointMat(config_setting_.imgRow, config_setting_.imgCol, CV_32F, cv::Scalar::all(INSIGNIFICANCE));
    point2img(cloud_obj, config_setting_.imgRow, config_setting_.imgCol, grid_data, pointMat);
    // seg the trunk
    cv::Mat pointTypeMat = cv::Mat::zeros(config_setting_.imgRow, config_setting_.imgCol, CV_32F);
    segPoints(pointMat, config_setting_.imgRow, config_setting_.imgCol, pointTypeMat, config_setting_);
    // image trans to points
    pcl::PointCloud<pcl::PointXYZ>::Ptr outdata(new pcl::PointCloud<pcl::PointXYZ>);
    img2point(grid_data, pointTypeMat, outdata);
    std::cout << "outdata: " << outdata->size() << std::endl;

    // cluster the object point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obj_cluster_points;    
    fec_cluster(outdata, obj_cluster_points, config_setting_);

    // // the next cluster
    // for(int i=0; i<obj_cluster_points.size(); i++)
    // {
    //     *cloud_obj += *obj_cluster_points[i];
    // }
    // obj_cluster_points.clear();
    // fec_cluster(outdata, obj_cluster_points, config_setting_);

    cluster_attributes(obj_clusters, discarded_clusters, obj_cluster_points, config_setting_);
    auto t3 = std::chrono::high_resolution_clock::now();
    cloud_obj->clear();
    obj_cluster_points.clear();

    auto t4 = std::chrono::high_resolution_clock::now();

    std::cout << "[Cluster NUM] obj_clusters: " << obj_clusters.size() << std::endl;
    
    std::cout << "[Time] Total: " << time_inc(t4, t1) << " ms"
              << " \n CSF: " << time_inc(t2, t1) << " ms"
              << " \n FEC for obj: " << time_inc(t3, t2) << " ms"
              << " \n Grid for gnd: " << time_inc(t4, t3) << " ms"
              << std::endl;
}

// init the cluster 
void GTINDescManager::init_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obj(new pcl::PointCloud<pcl::PointXYZ>);
    
    unsigned long size;
    size = input_cloud->size();
    // std::cout << "Total PointCloud has: " << size << " points." << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();
    // input point cloud to img
    cv::Mat pointMat(config_setting_.imgRow, config_setting_.imgCol, CV_32F, cv::Scalar::all(INSIGNIFICANCE));
    std::vector<std::vector<POINT_D>> grid_data(config_setting_.imgRow, std::vector<POINT_D>(config_setting_.imgCol));
    point2img(input_cloud, config_setting_.imgRow, config_setting_.imgCol, grid_data, pointMat);
    pointMat_vec_.push_back(pointMat);
    
    // seg the trunk
    cv::Mat pointTypeMat = cv::Mat::zeros(config_setting_.imgRow, config_setting_.imgCol, CV_32F);
    segPoints(pointMat, config_setting_.imgRow, config_setting_.imgCol, pointTypeMat, config_setting_);   
    pointTypeMat_vec_.push_back(pointTypeMat);
    
    // image trans to points (obj)
    img2point(grid_data, pointTypeMat, cloud_obj);
    ground = *cloud_ground;
    offground = *cloud_obj;
    auto t2 = std::chrono::high_resolution_clock::now();

    // cluster the trunks and calculate the attributes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obj_cluster_points;    
    fec_cluster(cloud_obj, obj_cluster_points, config_setting_);
    cluster_attributes(obj_clusters, discarded_clusters, obj_cluster_points, config_setting_);
    auto t3 = std::chrono::high_resolution_clock::now();
    std::cout << "obj_clusters: " << obj_clusters.size() <<std::endl;
    find_cluster_root(input_cloud, obj_clusters);
    
    auto t4 = std::chrono::high_resolution_clock::now();

    cloud_obj->clear();
    obj_cluster_points.clear();

    std::cout << "[Cluster NUM] obj_clusters: " << obj_clusters.size() << std::endl;
    
    std::cout << "[Time] Total: " << time_inc(t4, t1) << " ms"
              << " \n Seg: " << time_inc(t2, t1) << " ms"
              << " \n FEC for obj: " << time_inc(t3, t2) << " ms"
              << " \n Find cluster root: " << time_inc(t4, t3) << " ms"
              << std::endl;
}

// find the root point corresponding to the cluster
void GTINDescManager::find_cluster_root(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                        std::vector<Cluster> &clusters)
{
    int grid_num = config_setting_.grid_num;
    double range = config_setting_.range;
    std::vector<std::vector<double>> hori_grid(grid_num*2, std::vector<double>(grid_num*2, MAXFLOAT));

    double grid_size = range/grid_num;
    int NUM = input_cloud->size();

    // std::cout << BOLDRED << "Ori Point NUM: " << NUM << RESET << std::endl;
    // loop all point cloud, and find the minmum height in the grid
    for (int i=0; i<NUM; i++)
    {
        pcl::PointXYZ point = input_cloud->points[i];
        if(abs(point.x)<=range && abs(point.y)<=range)
        {
            int indxX, indxY;
            indxX = int(point.x / grid_size)+grid_num;
            indxY = int(point.y / grid_size)+grid_num;

            if(point.z < hori_grid[indxX][indxY])
            {
                hori_grid[indxX][indxY] = point.z;
            }
        }
    }
    // find the root points
    for(int i=0; i<clusters.size(); i++)
    {
        int indxX, indxY;
        indxX = int(clusters[i].center_[0] / grid_size) + grid_num;
        indxY = int(clusters[i].center_[1] / grid_size) + grid_num;
        // std::cout << "indxX: " << indxX << "indxY: " << indxY << std::endl;
        if(indxX >= 0 && indxX < grid_num*2 &&
            indxY >= 0 && indxY < grid_num*2)
        {
            clusters[i].root = hori_grid[indxX][indxY];
            
            // clusters[i].center_[2] = hori_grid[indxX][indxY];
            // clusters[i].p_center_.z = clusters[i].center_[2];
            // std::cout << BOLDRED << "Find the minmum hight: " << clusters[i].center_[2] << RESET << std::endl;
        }
        else
        {
            // std::cout << BOLDRED << "failed index" << RESET << std::endl;
            clusters.erase(clusters.begin() + i);
        }
    }
}

// get the center points and corresponding cluster points (Total)
void GTINDescManager::getPoint(FrameInfo &curr_frame_info)
{
    // get the total cluster center points, and points
    if(obj_clusters.size() == 0)
    {
        std::cout << "The clustes NUM is 0, so return." << std::endl;
        return;
    }
    // update the z value and add to the fix center
    for(auto iter=obj_clusters.begin(); iter!=obj_clusters.end(); iter++)
    {
        // get the init center point, default is the minmum z of each cluster
        pcl::PointXYZINormal p = iter->p_center_;
        
        if(config_setting_.centerSelection == 0)
        { // default
            curr_frame_info.currCenter->push_back(p);
            *(curr_frame_info.currPoints) += iter->points_;
            // same to the ori center point
            curr_frame_info.currCenterFix->push_back(p);
        }
        else if(config_setting_.centerSelection == 1)
        { // use the lowest point in the grid

            // change the z value of center point, then push back to the currCenterFix
            iter->p_center_.z = iter->root;
            iter->center_[2] = iter->root;
            curr_frame_info.currCenter->push_back(iter->p_center_);
            *(curr_frame_info.currPoints) += iter->points_;
            
            // ori center point
            curr_frame_info.currCenterFix->push_back(p);
        }
        else if(config_setting_.centerSelection == 2)
        {
            // change the z value of center point to 1.5
            iter->p_center_.z = 1.5;
            iter->center_[2] = 1.5;
            curr_frame_info.currCenter->push_back(iter->p_center_);
            *(curr_frame_info.currPoints) += iter->points_;
            
            // use the root poit as the Z coordinate
            p.z = iter->root;
            curr_frame_info.currCenterFix->push_back(p);
        }
    }
}

// construct the descriptors
void GTINDescManager::build_stdesc(FrameInfo &curr_frame_info)
{
    curr_frame_info.desc_.clear();
    double scale = 1.0 / config_setting_.side_resolution;
    // parameters, nearest corner points, max and min length of triangle edges
    int near_num = config_setting_.descriptor_near_num;
    double max_dis_threshold = config_setting_.descriptor_max_len;
    double min_dis_threshold = config_setting_.descriptor_min_len;
    std::unordered_map<UNI_VOXEL_LOC, bool> feat_map;
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kd_tree(
                            new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
    
    kd_tree->setInputCloud(curr_frame_info.currCenter);
    std::vector<int> pointIdxNKNSearch(near_num); // near indexs
    std::vector<float> pointNKNSquaredDistance(near_num); // near distance
    
    // loop all plane center points 
    for (size_t i = 0; i < curr_frame_info.currCenter->size(); i++) 
    {
        pcl::PointXYZINormal searchPoint = curr_frame_info.currCenter->points[i];
        if (kd_tree->nearestKSearch(searchPoint, near_num, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            // find two points to form a triangle
            for (int m = 1; m < near_num - 1; m++) 
            {
                for (int n = m + 1; n < near_num; n++) 
                {
                    // for triangle vertexes
                    pcl::PointXYZINormal p1 = searchPoint;
                    pcl::PointXYZINormal p2 = curr_frame_info.currCenter->points[pointIdxNKNSearch[m]];
                    pcl::PointXYZINormal p3 = curr_frame_info.currCenter->points[pointIdxNKNSearch[n]];
                    // length of all sides , in three dimensional
                    double a = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                                    pow(p1.z - p2.z, 2));
                    double b = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2) +
                                    pow(p1.z - p3.z, 2));
                    double c = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2) +
                                    pow(p3.z - p2.z, 2));
                    if (a > max_dis_threshold || b > max_dis_threshold ||
                        c > max_dis_threshold || a < min_dis_threshold ||
                        b < min_dis_threshold || c < min_dis_threshold) {
                        continue;
                    }
                    // re-range the vertex by the side length, in other words, sort the side length
                    double temp;
                    Eigen::Vector3d A, B, C;
                    Eigen::Vector3i l1, l2, l3;
                    Eigen::Vector3i l_temp;
                    l1 << 1, 2, 0;
                    l2 << 1, 0, 3;
                    l3 << 0, 2, 3;
                    // exchange the length and the index, make sure b > a
                    if (a > b) { 
                        temp = a;
                        a = b;
                        b = temp;
                        l_temp = l1;
                        l1 = l2;
                        l2 = l_temp;
                    }
                    // exchange the length and index, c>b
                    if (b > c) {
                        temp = b;
                        b = c;
                        c = temp;
                        l_temp = l2;
                        l2 = l3;
                        l3 = l_temp;
                    }
                    // exchange the length and index, b(c)>a again
                    if (a > b) {
                        temp = a;
                        a = b;
                        b = temp;
                        l_temp = l1;
                        l1 = l2;
                        l2 = l_temp;
                    }
                    // check augnmentation, (m) to (cm)
                    pcl::PointXYZ d_p;
                    d_p.x = a * 1000;
                    d_p.y = b * 1000;
                    d_p.z = c * 1000;
                    UNI_VOXEL_LOC position((int64_t)d_p.x, (int64_t)d_p.y, (int64_t)d_p.z);
                    auto iter = feat_map.find(position);
                    Eigen::Vector3d normal_1, normal_2, normal_3;
                    // whether have equal length triangle, make sure the triangle is unique
                    if (iter == feat_map.end())
                    {   
                        // no equal triangle has been added before that
                        // define the vertices of triangle
                        Eigen::Vector3d vertex_attached;
                        // A
                        if (l1[0] == l2[0]) {
                        A << p1.x, p1.y, p1.z;
                        normal_1 << p1.normal_x, p1.normal_y, p1.normal_z;
                        vertex_attached[0] = p1.intensity;
                        } else if (l1[1] == l2[1]) {
                        A << p2.x, p2.y, p2.z;
                        normal_1 << p2.normal_x, p2.normal_y, p2.normal_z;
                        vertex_attached[0] = p2.intensity;
                        } else {
                        A << p3.x, p3.y, p3.z;
                        normal_1 << p3.normal_x, p3.normal_y, p3.normal_z;
                        vertex_attached[0] = p3.intensity;
                        }
                        // B
                        if (l1[0] == l3[0]) {
                        B << p1.x, p1.y, p1.z;
                        normal_2 << p1.normal_x, p1.normal_y, p1.normal_z;
                        vertex_attached[1] = p1.intensity;
                        } else if (l1[1] == l3[1]) {
                        B << p2.x, p2.y, p2.z;
                        normal_2 << p2.normal_x, p2.normal_y, p2.normal_z;
                        vertex_attached[1] = p2.intensity;
                        } else {
                        B << p3.x, p3.y, p3.z;
                        normal_2 << p3.normal_x, p3.normal_y, p3.normal_z;
                        vertex_attached[1] = p3.intensity;
                        }
                        // C
                        if (l2[0] == l3[0]) {
                        C << p1.x, p1.y, p1.z;
                        normal_3 << p1.normal_x, p1.normal_y, p1.normal_z;
                        vertex_attached[2] = p1.intensity;
                        } else if (l2[1] == l3[1]) {
                        C << p2.x, p2.y, p2.z;
                        normal_3 << p2.normal_x, p2.normal_y, p2.normal_z;
                        vertex_attached[2] = p2.intensity;
                        } else {
                        C << p3.x, p3.y, p3.z;
                        normal_3 << p3.normal_x, p3.normal_y, p3.normal_z;
                        vertex_attached[2] = p3.intensity;
                        }
                        // the vetecs, center, attached attribute, slide length, and angle
                        TriDesc single_descriptor;
                        single_descriptor.vertex_A_ = A;
                        single_descriptor.vertex_B_ = B;
                        single_descriptor.vertex_C_ = C;
                        single_descriptor.center_ = (A + B + C) / 3;
                        single_descriptor.vertex_attached_ = vertex_attached;
                        single_descriptor.side_length_ << scale * a, scale * b, scale * c;
                        single_descriptor.angle_[0] = fabs(5 * normal_1.dot(normal_2));
                        single_descriptor.angle_[1] = fabs(5 * normal_1.dot(normal_3));
                        single_descriptor.angle_[2] = fabs(5 * normal_3.dot(normal_2));
                        // single_descriptor.angle << 0, 0, 0;
                        single_descriptor.frame_id_ = current_frame_id_;
                        Eigen::Matrix3d triangle_positon;
                        feat_map[position] = true;
                        curr_frame_info.desc_.push_back(single_descriptor);
                    }
                }
            }
        }
    }
}

// select the candidate descriptors, according to the edge length
void GTINDescManager::candidate_frames_selector(const FrameInfo &curr_frame,
                                               std::vector<TriMatchList> &candidate_matcher_vec) 
{
    int descNum = curr_frame.desc_.size();
    double match_array[MAX_FRAME_N] = {0};
    std::vector<std::pair<TriDesc, TriDesc>> match_vec;
    std::vector<int> match_index_vec;
    double currFrameID = curr_frame.frame_id_;
    // near position in 6 directions, 3*3*3
    std::vector<Eigen::Vector3i> voxel_round;
    for (int x = -1; x <= 1; x++) 
    {
        for (int y = -1; y <= 1; y++) 
        {
            for (int z = -1; z <= 1; z++) 
            {
                Eigen::Vector3i voxel_inc(x, y, z);
                voxel_round.push_back(voxel_inc);
            }
        }
    }

    // some useful variables
    std::vector<size_t> index(descNum);
    std::vector<bool> useful_match(descNum);
    std::vector<std::vector<size_t>> useful_match_index(descNum);
    std::vector<std::vector<TriDesc_LOC>> useful_match_position(descNum);
    for (size_t i = 0; i < index.size(); ++i) 
    {
        index[i] = i;
        useful_match[i] = false;
    }
    
    // speed up matching
    int dis_match_cnt = 0;
    int final_match_cnt = 0;
    // loop all triangles in the vector, and find the near positions in the database
    // #pragma omp parallel for num_threads(8)
    for (size_t i = 0; i < descNum; i++) 
    {
        TriDesc src_std = curr_frame.desc_[i];
        TriDesc_LOC position;
        int best_index = 0;
        TriDesc_LOC best_position;
        double dis_threshold = src_std.side_length_.norm() * config_setting_.rough_dis_threshold; 
        // std::cout << "side_length_.norm(): " << src_std.side_length_.norm()
        //           << ", rough_dis_threshold: " << config_setting_.rough_dis_threshold
        //           << ", dis_threshold: " << dis_threshold << std::endl;
        // loop all round voxels, find the nearest position in integrator type
        for (auto voxel_inc : voxel_round) 
        {
            // move in six directions
            position.x = (int)(src_std.side_length_[0] + voxel_inc[0]);
            position.y = (int)(src_std.side_length_[1] + voxel_inc[1]);
            position.z = (int)(src_std.side_length_[2] + voxel_inc[2]);
            // this center is get from the side length of triangles
            Eigen::Vector3d voxel_center((double)position.x + 0.5,
                                        (double)position.y + 0.5,
                                        (double)position.z + 0.5);
            if ((src_std.side_length_ - voxel_center).norm() < 1.5) 
            {
                auto iter = data_base_.find(position);
                if (iter != data_base_.end())
                {   // found the corresponding triangle in data base
                    for (size_t j = 0; j < data_base_[position].size(); j++)
                    {   
                        // distance between source and candidate triangle
                        double dis = (src_std.side_length_ - data_base_[position][j].side_length_).norm();
                        double diffID = currFrameID - data_base_[position][j].frame_id_;
                        // rough filter with side lengths
                        // std::cout << "dis: " << dis << "  dis_threshold: " << dis_threshold << std::endl;
                        // matched by distance
                        if (dis < dis_threshold && diffID != 0) 
                        {
                            dis_match_cnt++; 
                            double vex_diff = (src_std.vertex_attached_ - 
                                               data_base_[position][j].vertex_attached_).norm();
                            // the difference of vex type, e.g., line or plane
                            if(vex_diff <= config_setting_.vertex_diff_threshold)
                            {
                                final_match_cnt++;
                                useful_match[i] = true;
                                useful_match_position[i].push_back(position);
                                useful_match_index[i].push_back(j);
                            }
                        }
                    }
                }
            }
        }
    }
    
    // std::cout << "******final_match_cnt: " << final_match_cnt 
    //           << ", dis_match_cnt: " << dis_match_cnt << std::endl;
    
    // record match index
    std::vector<Eigen::Vector2i> index_recorder;
    for (size_t i = 0; i < useful_match.size(); i++) 
    {
        if (useful_match[i]) 
        {
            for (size_t j = 0; j < useful_match_index[i].size(); j++) 
            {   
                // vote for the frame id
                match_array[data_base_[useful_match_position[i][j]]
                                      [useful_match_index[i][j]].frame_id_] += 1;
                // record index of triangle, and the index of triangle in fixed position
                Eigen::Vector2i match_index(i, j);
                index_recorder.push_back(match_index);
                match_index_vec.push_back(
                    data_base_[useful_match_position[i][j]][useful_match_index[i][j]].frame_id_);
            }
        }
    }

    // select candidate according to the matching score
    for (int cnt = 0; cnt < config_setting_.candidate_num; cnt++) 
    {
        // select the max vote(fist cnt num) and its index
        double max_vote = 1;
        int max_vote_index = -1;

        // select the index with max vote
        for (int i = 0; i < MAX_FRAME_N; i++) 
        {
            if (match_array[i] > max_vote) 
            {
                max_vote = match_array[i];
                max_vote_index = i;
            }
        }

        // have found the max votes
        TriMatchList match_triangle_list;
        if (max_vote_index >= 0 && max_vote >= 5) 
        {
            // remove this, set votes=0
            match_array[max_vote_index] = 0; 
            // record the frame id, including current and the candidate
            match_triangle_list.match_id_.first = curr_frame.frame_id_;
            match_triangle_list.match_id_.second = max_vote_index;
            for (size_t i = 0; i < index_recorder.size(); i++) 
            {   // caculated index and the max vote index has been found
                if (match_index_vec[i] == max_vote_index) 
                {
                    std::pair<TriDesc, TriDesc> single_match_pair;
                    // the search triangle
                    single_match_pair.first = curr_frame.desc_[index_recorder[i][0]];
                    // the candidate triangle
                    single_match_pair.second =
                        data_base_[useful_match_position[index_recorder[i][0]][index_recorder[i][1]]]
                                  [useful_match_index[index_recorder[i][0]][index_recorder[i][1]]];
                    // push back in the match list
                    match_triangle_list.match_list_.push_back(single_match_pair);
                }
            }
            candidate_matcher_vec.push_back(match_triangle_list);
        } 
        else 
        {
            break;
        }
    }
}

// Get the best candidate frame by geometry check
void GTINDescManager::candidate_frames_verify( const FrameInfo &curr_frame,
    const TriMatchList &candidate_matcher, double &verify_score,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
    std::vector<std::pair<TriDesc, TriDesc>> &sucess_match_vec,
    ConfigSetting &config_setting) 
{
    sucess_match_vec.clear();
    // not all matched triangle was used to verify, only the "use_size" are used
    // the interval length
    int skip_len = (int)(candidate_matcher.match_list_.size() / config_setting.use_matched_num) + 1;
    // used size, the num of triangles
    skip_len = 1;
    int use_size = candidate_matcher.match_list_.size() / skip_len;
    std::cout << "***************************use_size: " << use_size << ", skip_len: "<< skip_len << std::endl;
    // double dis_threshold = 1.0;
    double dis_threshold = config_setting.dist_candi_frames_verify;
    std::cout << "***************************dis_threshold: " << dis_threshold << std::endl;
    std::vector<size_t> index(use_size);
    std::vector<int> vote_list(use_size);
    for (size_t i = 0; i < index.size(); i++) {
        index[i] = i;
    }
    
    std::mutex mylock;
    // loop the selected triangles in match list, num is use_size
    #pragma omp parallel for num_threads(8)
    for (size_t i = 0; i < use_size; i++) 
    {
        // single triangle pair
        auto single_pair = candidate_matcher.match_list_[i * skip_len];
        int vote = 0;
        Eigen::Matrix3d test_rot;
        Eigen::Vector3d test_t;
        // solve the test pose by using a single triangle
        triangle_solver(single_pair, test_t, test_rot);
        // loop all triangles in the match list
        for (size_t j = 0; j < candidate_matcher.match_list_.size(); j++) 
        {
            auto verify_pair = candidate_matcher.match_list_[j];
            Eigen::Vector3d A = verify_pair.first.vertex_A_;
            Eigen::Vector3d A_transform = test_rot * A + test_t;
            Eigen::Vector3d B = verify_pair.first.vertex_B_;
            Eigen::Vector3d B_transform = test_rot * B + test_t;
            Eigen::Vector3d C = verify_pair.first.vertex_C_;
            Eigen::Vector3d C_transform = test_rot * C + test_t;
            double dis_A = (A_transform - verify_pair.second.vertex_A_).norm();
            double dis_B = (B_transform - verify_pair.second.vertex_B_).norm();
            double dis_C = (C_transform - verify_pair.second.vertex_C_).norm();
            if (dis_A < dis_threshold && dis_B < dis_threshold && dis_C < dis_threshold) 
            {
                // vote for the test pose
                vote++;
            }
        }
        mylock.lock();
        vote_list[i] = vote;
        mylock.unlock();
    }
    // find the max votes and the corresponding index 
    int max_vote_index = 0;
    int max_vote = 0;
    for (size_t i = 0; i < vote_list.size(); i++) 
    {
        if (max_vote < vote_list[i]) 
        {
            max_vote_index = i; // this index is not real, it have been devided by skip_len before
            max_vote = vote_list[i];
        }
    }
    std::cout << "***************************max_vote: " << max_vote << std::endl;
    // the triangle with maxmium votes
    if (max_vote >= 3) 
    {
        // the best pair, and use it to calculate best pose
        auto best_pair = candidate_matcher.match_list_[max_vote_index * skip_len];
        int vote = 0;
        Eigen::Matrix3d best_rot;
        Eigen::Vector3d best_t;
        triangle_solver(best_pair, best_t, best_rot);
        relative_pose.first = best_t;
        relative_pose.second = best_rot;
        // loop all triangles, and get the sucess matched triangles
        for (size_t j = 0; j < candidate_matcher.match_list_.size(); j++) 
        {
            auto verify_pair = candidate_matcher.match_list_[j];
            Eigen::Vector3d A = verify_pair.first.vertex_A_;
            Eigen::Vector3d A_transform = best_rot * A + best_t;
            Eigen::Vector3d B = verify_pair.first.vertex_B_;
            Eigen::Vector3d B_transform = best_rot * B + best_t;
            Eigen::Vector3d C = verify_pair.first.vertex_C_;
            Eigen::Vector3d C_transform = best_rot * C + best_t;
            double dis_A = (A_transform - verify_pair.second.vertex_A_).norm();
            double dis_B = (B_transform - verify_pair.second.vertex_B_).norm();
            double dis_C = (C_transform - verify_pair.second.vertex_C_).norm();
            if (dis_A < dis_threshold && dis_B < dis_threshold && dis_C < dis_threshold) 
            {
                // transform the triangle descriptor
                verify_pair.first.vertex_A_ = A_transform;
                verify_pair.first.vertex_B_ = B_transform;
                verify_pair.first.vertex_C_ = C_transform;
                // get the sucess matched triangles
                sucess_match_vec.push_back(verify_pair);
            }
        }
        std::cout << "***************************sucess_match_vec: " << sucess_match_vec.size() << std::endl;
        
        if(config_setting.centerSelection == 0 || config_setting.centerSelection == 1)
        {
            verify_score = geometric_verify(
                curr_frame.currCenter,
                frame_info_vec_[candidate_matcher.match_id_.second].currCenter, relative_pose);
        }
        else if(config_setting.centerSelection == 2)
        {
            verify_score = geometric_verify(
                curr_frame.currCenterFix,
                frame_info_vec_[candidate_matcher.match_id_.second].currCenterFix, relative_pose);
        }
        // std::cout << "first: " << candidate_matcher.match_id_.first << " second: " << candidate_matcher.match_id_.second << std::endl;
    }
    else
    {
        verify_score = -1;
    }
}

// solve the transform pose between two Triangle desciptors, by SVD
void GTINDescManager::triangle_solver(std::pair<TriDesc, TriDesc> &std_pair,
                                    Eigen::Vector3d &t, Eigen::Matrix3d &rot) 
{
    Eigen::Matrix3d src = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d ref = Eigen::Matrix3d::Zero();
    // centerlize each vetex of triangle
    src.col(0) = std_pair.first.vertex_A_ - std_pair.first.center_;
    src.col(1) = std_pair.first.vertex_B_ - std_pair.first.center_;
    src.col(2) = std_pair.first.vertex_C_ - std_pair.first.center_;
    ref.col(0) = std_pair.second.vertex_A_ - std_pair.second.center_;
    ref.col(1) = std_pair.second.vertex_B_ - std_pair.second.center_;
    ref.col(2) = std_pair.second.vertex_C_ - std_pair.second.center_;
    Eigen::Matrix3d covariance = src * ref.transpose(); // matrix, 3*3
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d U = svd.matrixU();
    // calculate the Rot and Trans
    rot = V * U.transpose();
    if (rot.determinant() < 0) 
    {
        Eigen::Matrix3d K;
        K << 1, 0, 0, 0, 1, 0, 0, 0, -1;
        rot = V * K * U.transpose();
    }
    t = -rot * std_pair.first.center_ + std_pair.second.center_;    
}

// geometric verify, the distance of point_to_plane, and the normal difference
double GTINDescManager::geometric_verify(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform) 
{
    Eigen::Vector3d t = transform.first;
    Eigen::Matrix3d rot = transform.second;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // target point cloud is inputed to the kd Tree
    for (size_t i = 0; i < target_cloud->size(); i++) 
    {
        pcl::PointXYZ pi;
        pi.x = target_cloud->points[i].x;
        pi.y = target_cloud->points[i].y;
        pi.z = target_cloud->points[i].z;
        input_cloud->push_back(pi);
    }
    kd_tree->setInputCloud(input_cloud);
    
    int K = 3;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    // loop all source points, and verfey the normal and distance
    double useful_match = 0;
    double normal_threshold = config_setting_.normal_geo_verify;
    double dis_threshold = config_setting_.dis_geo_verify;
    double diff_z = 0;
    for (size_t i = 0; i < source_cloud->size(); i++) 
    {
        pcl::PointXYZINormal searchPoint = source_cloud->points[i];
        pcl::PointXYZ use_search_point;
        Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
        pi = rot * pi + t;
        // use the transformed "point" for searching
        use_search_point.x = pi[0];
        use_search_point.y = pi[1];
        use_search_point.z = pi[2];
        Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                    searchPoint.normal_z);
        ni = rot * ni;
        // find three nearest points
        if (kd_tree->nearestKSearch(use_search_point, K, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) 
        {
            for (size_t j = 0; j < K; j++) 
            {
                pcl::PointXYZINormal nearstPoint = target_cloud->points[pointIdxNKNSearch[j]];
                Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
                Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y, nearstPoint.normal_z);
                Eigen::Vector3d normal_inc = ni - tni;
                Eigen::Vector3d normal_add = ni + tni;
                // the distance between search point to the nearest plane/line
                double point_to_target;
                if(searchPoint.intensity != nearstPoint.intensity)
                    continue;
                else if(nearstPoint.intensity = 0)
                {
                    point_to_target = fabs(tni.transpose() * (pi - tpi));
                    // double dxy = sqrt((pi - tpi)[0]*(pi - tpi)[0] + (pi - tpi)[1]*(pi - tpi)[1]);
                    // std::cout << "point_to_target: " << point_to_target << ", dxy: " << dxy << std::endl;
                    if (point_to_target < dis_threshold) 
                    {
                        // find the correct match, then break down
                        useful_match++;
                        break;
                    }
                } 
                else if(nearstPoint.intensity = 1)
                {
                    point_to_target = ((pi - tpi).cross(tni)).norm()/2;
                    // double dxy = sqrt((pi - tpi)[0]*(pi - tpi)[0] + (pi - tpi)[1]*(pi - tpi)[1]);
                    // std::cout << "point_to_target: " << point_to_target << ", dxy: " << dxy << std::endl;
                    if ((normal_inc.norm() < normal_threshold ||normal_add.norm() < normal_threshold) 
                        && point_to_target < dis_threshold)
                    {
                        // find the correct match, then break down
                        useful_match++;
                        // sum the difference between source and target points
                        diff_z += (tpi[2] - pi[2]);
                        break;
                    }
                }
            }
        }
    }
    
    std::cout << "*****useful_match: " << useful_match << "*****total: " << source_cloud->size()<< std::endl;
    std::cout << "*****diff Z: " << diff_z/useful_match << std::endl;
    
    // update the relative transform
    if(config_setting_.centerSelection == 0 || config_setting_.centerSelection == 1)
    {
        // transform.first[2] -= diff_z/useful_match;
        transform.first[2] = transform.first[2];
    }
    else if(config_setting_.centerSelection == 2)
    {
        transform.first[2] = diff_z/useful_match;
    }
    
    // the correct ratio in the source point 
    return useful_match / source_cloud->size();   
}
/********************USEFUL FUNCTIONS**********************/

