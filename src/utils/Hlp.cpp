#include "math.h"
#include "float.h"
#include <unordered_map>

#ifndef _HLP_H_Included_
#define _HLP_H_Included_
#include "../include/utils/Hlp.h"
#endif

// Associated functions
// int round(double x)
// {
//    return int(x > 0.0 ? x + 0.5 : x - 0.5);
// }

// 分配数组内存
bool New_Double(double** pdArray, long lEleNum)
{
	*pdArray = new double[lEleNum];
	if( *pdArray )	{
		return true;
	}
	else	{
		return false;
	}
}
bool New_Int(int** pnArray, long lEleNum)
{
	*pnArray = new int[lEleNum];
	if( *pnArray )	{
		return true;
	}
	else	{
		return false;
	}
}
bool New_Long(long** plArray, long lEleNum)
{
	*plArray = new long[lEleNum];
	if( *plArray )	{
		return true;
	}
	else	{
		return false;
	}
}
bool New_Char(char** pcArray, long lEleNum)
{
	*pcArray = new char[lEleNum];
	if( *pcArray )	{
		return true;
	}
	else	{
		return false;
	}
}
bool New_Bool(bool** pbArray, long lEleNum)
{
	*pbArray = new bool[lEleNum];
	if( *pbArray )	{
		return true;
	}
	else	{
		return false;
	}
}

// 删除内存空间
bool Del_Double(double** pdArray)
{
	if ( *pdArray )
	{
		delete[] *pdArray;
		*pdArray = NULL;
		return true;
	}
	else
	{
		return false;
	}
}
bool Del_Int(int** pnArray)
{
	if ( *pnArray )
	{
		delete[] *pnArray;
		*pnArray = NULL;
		return true;
	}
	else
	{
		return false;
	}
}
bool Del_Long(long** plArray)
{
	if ( *plArray )
	{
		delete[] *plArray;
		*plArray = NULL;
		return true;
	}
	else
	{
		return false;
	}
}
bool Del_Char(char** pcArray)
{
	if ( *pcArray )
	{
		delete[] *pcArray;
		*pcArray = NULL;
		return true;
	}
	else
	{
		return false;
	}
}
bool Del_Bool(bool** pbArray)
{
	if ( *pbArray )
	{
		delete[] *pbArray;
		*pbArray = NULL;
		return true;
	}
	else
	{
		return false;
	}
}

// 统一设置数组的值
void SetVal_Int(int* pnArray, long lEleNum, int nVal)
{
	for(long i = 0; i < lEleNum; i++)
		*(pnArray + i) = nVal; 	
}
void SetVal_Long(long *plArray, long lEleNum, long lVal)
{
	for(long i = 0; i < lEleNum; i++)
		*(plArray + i) = lVal; 
}
void SetVal_Double(double *pdArray, long lEleNum, double dVal)
{
	for(long i = 0; i < lEleNum; i++)
		*(pdArray + i) = dVal; 
}
void SetVal_Bool(bool *pbArray, long lEleNum, bool bVal)
{
	for(long i = 0; i < lEleNum; i++)
		*(pbArray + i) = bVal; 
}

// 判断两个值是否相等
bool Val_Equ(const int nVal_1, const int nVal_2)
{
    double dDif = (double)( nVal_1 - nVal_2 );
	dDif = fabs( dDif );
    if ( DBL_EPSILON > dDif )
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool Val_Equ(const double bVal_1, const double bVal_2)
{
    double dDif = fabs( bVal_1 - bVal_2 );
    if ( DBL_EPSILON > dDif )
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool Val_Equ(const long lVal_1, const long lVal_2)
{
    long lDiff = abs( lVal_1 - lVal_2 );
    if ( 1 > lDiff )
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool Val_Equ(const char& cVal_1, const char& cVal_2)
{
    if ( cVal_1 == cVal_2 )
    {
        return true;
    }
    else
    {
        return false;
    }
}


long ArrayMax(const double *pdIn, const long lEle_Num)
{
	long lMax_ID = 0;
	double dMax  = *pdIn;
	for(long lEle = 0; lEle < lEle_Num; lEle++)
	{
		if ( *(pdIn + lEle) > dMax )
		{
			dMax    = *(pdIn + lEle);
			lMax_ID = lEle;
		}
	}
	return lMax_ID;
}
long ArrayMax(const long *plIn, const long lEle_Num)
{
	long lMax_ID = 0;
	long lMax    = *plIn;
	for(long lEle = 0; lEle < lEle_Num; lEle++)
	{
		if ( *(plIn + lEle) > lMax )
		{
			lMax    = *(plIn + lEle);
			lMax_ID = lEle;
		}
	}
	return lMax_ID;
}
long ArrayMax(const int *pnIn, const long lEle_Num)
{
	long lMax_ID = 0;
	int nMax     = *pnIn;
	for(long lEle = 0; lEle < lEle_Num; lEle++)
	{
		if ( *(pnIn + lEle) > nMax )
		{
			nMax    = *(pnIn + lEle);
			lMax_ID = lEle;
		}
	}
	return lMax_ID;
}

// read the parameters from yaml file 
void ReadParas(const std::string& file_path, ConfigSetting &config_setting)
{
	cv::FileStorage fs(file_path, cv::FileStorage::READ);

	// read the parameters for project operation
	config_setting.imgRow = (int)fs["imgRow"];
	config_setting.imgCol = (int)fs["imgCol"];
	std::cout << BOLDBLUE << "-----------Read Projection Parameters-----------" << RESET << std::endl;
	std::cout << "imgRow: " << config_setting.imgRow << std::endl;
	std::cout << "imgCol: " << config_setting.imgCol << std::endl;
	
	// read the horizon parameters, for selection minmum Z
	config_setting.range = (double)fs["range"];
	config_setting.grid_num = (int)fs["grid_num"];
	std::cout << BOLDBLUE << "-----------Read Horizon Parameters-----------" << RESET << std::endl;
	std::cout << "range: " << config_setting.range << std::endl;
	std::cout << "grid_num: " << config_setting.grid_num << std::endl;
	
	// read the parameters for segment operation
	config_setting.isRow_Or_Column = (int)fs["isRow_Or_Column"];
	config_setting.dDif_Th = (double)fs["dDif_Th"];
	config_setting.nSch_Stp = (int)fs["nSch_Stp"];
	config_setting.lGrp_Ele_Min = (int)fs["lGrp_Ele_Min"];	
	std::cout << BOLDBLUE << "-----------Read Segmentation Parameters-----------" << RESET << std::endl;
	std::cout << "isRow_Or_Column: " << config_setting.isRow_Or_Column << std::endl;
	std::cout << "dDif_Th: " << config_setting.dDif_Th << std::endl;
	std::cout << "nSch_Stp: " << config_setting.nSch_Stp << std::endl;
	std::cout << "lGrp_Ele_Min: " << config_setting.lGrp_Ele_Min << std::endl;

	// read the parameters for FEC cluster
	config_setting.min_component_size = (int)fs["min_component_size"];
	config_setting.tolorance = (double)fs["tolorance"];
	config_setting.max_n = (int)fs["max_n"];
	config_setting.merge_dist = (double)fs["merge_dist"];	
	std::cout << BOLDBLUE << "-----------Read FEC cluster Parameters-----------" << RESET << std::endl;
	std::cout << "min_component_size: " << config_setting.min_component_size << std::endl;
	std::cout << "tolorance: " << config_setting.tolorance << std::endl;
	std::cout << "max_n: " << config_setting.max_n << std::endl;
	std::cout << "merge_dist: " << config_setting.merge_dist << std::endl;

	// read the parameters to distinguish the type of cluster
	config_setting.linearityThres = (double)fs["linearityThres"];
	config_setting.scateringThres = (double)fs["scateringThres"];
	config_setting.upThres = (double)fs["upThres"];
	config_setting.clusterHeight = (double)fs["clusterHeight"];
	config_setting.centerSelection = (int)fs["centerSelection"];
	std::cout << BOLDBLUE << "-----------Read Parameters to distinguish the type of cluster-----------" << RESET << std::endl;
	std::cout << "linearityThres: " << config_setting.linearityThres << std::endl;
	std::cout << "scateringThres: " << config_setting.scateringThres << std::endl;
	std::cout << "upThres: " << config_setting.upThres << std::endl;
	std::cout << "clusterHeight: " << config_setting.clusterHeight << std::endl;
	std::cout << "centerSelection: " << config_setting.centerSelection << std::endl;

	// for tri descriptor
	config_setting.descriptor_near_num = (int)fs["descriptor_near_num"];
	config_setting.descriptor_min_len = (double)fs["descriptor_min_len"];
	config_setting.descriptor_max_len = (double)fs["descriptor_max_len"];
	config_setting.descriptor_len_diff = (double)fs["descriptor_len_diff"];
	config_setting.side_resolution = (double)fs["side_resolution"];
	std::cout << BOLDBLUE << "-----------Read Parameters to triangle descriptor-----------" << RESET << std::endl;
	std::cout << "descriptor_near_num: " << config_setting.descriptor_near_num << std::endl;
	std::cout << "descriptor_min_len: " << config_setting.descriptor_min_len << std::endl;
	std::cout << "descriptor_max_len: " << config_setting.descriptor_max_len << std::endl;
	std::cout << "descriptor_len_diff: " << config_setting.descriptor_len_diff << std::endl;
	std::cout << "side_resolution: " << config_setting.side_resolution << std::endl;

	// read the parameters for registration
	config_setting.candidate_num = (int)fs["candidate_num"];
	config_setting.rough_dis_threshold = (double)fs["rough_dis_threshold"];
	config_setting.vertex_diff_threshold = (double)fs["vertex_diff_threshold"];
	config_setting.icp_threshold = (double)fs["icp_threshold"];
	config_setting.dist_candi_frames_verify = (double)fs["dist_candi_frames_verify"];
	config_setting.normal_geo_verify = (double)fs["normal_geo_verify"];
	config_setting.dis_geo_verify = (double)fs["dis_geo_verify"];
	config_setting.use_matched_num = (int)fs["use_matched_num"];
	std::cout << BOLDBLUE << "-----------Read Parameters to registration-----------" << RESET << std::endl;
	std::cout << "candidate_num: " << config_setting.candidate_num << std::endl;
	std::cout << "rough_dis_threshold: " << config_setting.rough_dis_threshold << std::endl;
	std::cout << "vertex_diff_threshold: " << config_setting.vertex_diff_threshold << std::endl;
	std::cout << "icp_threshold: " << config_setting.icp_threshold << std::endl;
	std::cout << "dist_candi_frames_verify: " << config_setting.dist_candi_frames_verify << std::endl;
	std::cout << "normal_geo_verify: " << config_setting.normal_geo_verify << std::endl;
	std::cout << "dis_geo_verify: " << config_setting.dis_geo_verify << std::endl;
	std::cout << "use_matched_num: " << config_setting.use_matched_num << std::endl;
}

// split the line into string
std::vector<std::string> split(std::string str,std::string s)
{
    boost::regex reg(s.c_str());
    std::vector<std::string> vec;
    boost::sregex_token_iterator it(str.begin(),str.end(),reg,-1);
    boost::sregex_token_iterator end;
    while(it!=end)
    {
        vec.push_back(*it++);
    }
    return vec;
}

// read the trans file from LeiCa RTC360
std::vector<Eigen::Affine3d> readTLSTrans(const std::string tansFile)
{
    std::vector<Eigen::Affine3d> trans_matrix;
    std::ifstream open_file(tansFile);
	if (!open_file.is_open()) 
	{
		// Error, and exit
    	std::cerr << "Error: Failed to open file: " << tansFile << std::endl;
    	return trans_matrix;
	}

    static int num = 0;
    if(open_file){
		std::string line;
        int i = 0, j = 0;
		while(!open_file.eof()){
			getline(open_file,line);

			if(line.length() != 0){
				if(line[0] == 'J'){
					getline(open_file,line);
					// 找到 “(” 与 “)”之间的数字
					size_t pos = line.find("(");
					line.erase(0, pos + 1);
					pos = line.find(")");
					line.erase(pos, line.length());

					// 分割 读取平移量
					// std::cout << BOLDBLUE << "t_line: " << line << RESET << std::endl;
					std::vector<std::string> vec=split(line, ", ");
					Eigen::Vector3d t;
					for(int i=0,size=vec.size();i<size;i++)
					{
						t(i) = stof(vec[i]);
						// std::cout<<vec[i]<<std::endl;
					}
					
					// 读取下一行，即为旋转量
					std::getline(open_file,line);
					std::string line1 = line;
					pos = line1.find("):");
					line1.erase(0, pos + 2);
					pos = line1.find(" deg");
					line1.erase(pos, line1.length());
					// std::cout << BOLDBLUE << "r_line: " << line1 << RESET << std::endl;
					double alpha = stof(line1);

					// 找到 “(” 与 “)”之间的数字
					pos = line.find("(");
					line.erase(0, pos + 1);
					pos = line.find(")");
					line.erase(pos, line.length());
					// 分割 读取旋转量
					// std::cout << BOLDBLUE << "r_line: " << line << RESET << std::endl;
					vec=split(line, ", ");
					Eigen::Vector3d r;
					for(int i=0,size=vec.size();i<size;i++)
					{
						r(i) = stof(vec[i]);
						// std::cout<<vec[i]<<std::endl;
					}

					// to affine structure
					Eigen::AngleAxisd rotation_vector(alpha*((M_PI / 180)),r);
					Eigen::Matrix3d rotation_matrix;
					rotation_matrix=rotation_vector.matrix();
					
					Eigen::Affine3d t_m = Eigen::Affine3d::Identity();
					t_m.rotate(rotation_matrix);
					t_m.translation() = t;
					// std::cout << t_m.matrix() << std::endl;
					trans_matrix.push_back(t_m);
					
					num++;
				}
			}
		}
	}

	std::cout << "line number: " << num << std::endl;
	return trans_matrix;
}

// read the trans file of Tongji Trees dataset
void readTongjiTrans(const std::string& filename, 
                     std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& effectivenessData,
                     std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& robustnessData,
                     std::vector<std::tuple<int, int, Eigen::Matrix4d>>& transformations)
{
	std::ifstream file(filename);
    std::string line;
    std::string currentSection;

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    while (std::getline(file, line)) {
        // 去除行首尾空白
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        if (line.empty() || line[0] == '#' || line.size()==1) {
            
            // 检查是否是新节
            if (line.find("Effectiveness Test") != std::string::npos) {
                currentSection = "effectiveness";
                continue;
            } else if (line.find("Robustness Test") != std::string::npos) {
                currentSection = "robustness";
                continue;
            }
            
            continue; // 跳过注释和空行
        }
        // std::cout << "currentSection: " << currentSection << std::endl;
        
        // 处理变换描述行
        std::string from, to, end;
        std::istringstream iss(line);
        if (!(iss >> from >> to >> end)) {
            std::cerr << "变换描述格式错误: " << line << std::endl;
            continue;
        }

        // 读取4x4变换矩阵
        Eigen::Matrix4d transformation;
        for (int i = 0; i < 4; ++i) {
            if (!std::getline(file, line)) {
                std::cerr << "读取矩阵时出现错误，行数不足。" << std::endl;
                return;
            }
            // 去除行首尾空白
            line.erase(0, line.find_first_not_of(" \t"));
            line.erase(line.find_last_not_of(" \t") + 1);
            if (line.empty()) {
                std::cerr << "读取到空行，期望第 " << i + 1 << " 行。" << std::endl;
                --i; // 重新读取这一行
                continue;
            }
            // std::cout << "line: " << line << std::endl;
            std::istringstream issMatrix(line);
            for (int j = 0; j < 4; ++j) {
                if (!(issMatrix >> transformation(i, j))) {
                    std::cerr << "矩阵数据格式错误，行: " << line << std::endl;
                    return;
                }
            }
        }
        // std::cout << "transformation: \n" << transformation << std::endl;
        
        // 提取旋转矩阵和平移向量
        Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
        Eigen::Vector3d translation = transformation.block<3, 1>(0, 3);

        // 将数据存入对应的向量
        if (currentSection == "effectiveness") {
            effectivenessData.emplace_back(translation, rotation);
            // std::cout << "添加到 Effectiveness: " << translation.transpose() << std::endl;
        } else if (currentSection == "robustness") {
            robustnessData.emplace_back(translation, rotation);
            // std::cout << "添加到 Robustness: " << translation.transpose() << std::endl;
        }

        // // 保存变换矩阵
		// std::cout << "transformation: \n" << transformation << std::endl;
        int first = std::stoi(from.substr(1));
        int second = std::stoi(end.substr(1));
		// std::cout << first << " to " << second << std::endl;
        transformations.emplace_back(first, second, transformation);
    }

    file.close();
}

void readFGITrans(const std::string filename, std::pair<Eigen::Vector3d, Eigen::Matrix3d>& trans)
{
	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();; // 4x4 矩阵
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
		return;
    }

 	std::string line;
    while (true) {
        // 逐行读取 4 行数据
        for (int i = 0; i < 4; ++i) {
            if (!std::getline(file, line)) {
                break; // 结束读取
            }
			// std::cout << "line: " << line << std::endl;
            std::istringstream rowStream(line);
            for (int j = 0; j < 4; ++j) {
                if (!(rowStream >> matrix(i, j))) {
                    std::cerr << "Error: Invalid matrix format in line: " << line << std::endl;
                }
            }
        }

        // 如果读取到的行数不足 4 行，则退出
        if (file.eof()) {
            break;
        }
    }
	// std::cout << "matrix: \n" << matrix << std::endl;
	// 提取旋转矩阵和平移向量
	Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
	Eigen::Vector3d translation = matrix.block<3, 1>(0, 3);

	trans.first = translation;
	trans.second = rotation;

	file.close();
}

// read the TLS point cloud data
void readTLSData(const std::string lasFile, pcl::PointCloud<pcl::PointXYZ>::Ptr &tlsPC)
{
    // creats las reader
    std::ifstream ifs(lasFile.c_str(), std::ios::in | std::ios::binary);
	
	if (!ifs.is_open()) {
        std::cerr << "Error: Could not open file " << lasFile << std::endl;
        return;
    }
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);	// read las file
	// the num of las file
    unsigned long int nbPoints=reader.GetHeader().GetPointRecordsCount();
    pcl::PointXYZ p;
	while(reader.ReadNextPoint()) 
	{
        // get the data from las (X Y Z)
		p.x = (reader.GetPoint().GetX());
	    p.y = (reader.GetPoint().GetY());
	    p.z = (reader.GetPoint().GetZ());

		tlsPC->points.push_back(p);		
	}
    // pcxyz type point cloud
    tlsPC->width    = nbPoints;
	tlsPC->height   = 1;			
	tlsPC->is_dense = false;
    // std::cout << "las NUM: " << nbPoints << ", pcd NUM: " << tlsPC->size() << std::endl;
}

// write final point cloud
void writeLas(std::string las_file, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
	std::ofstream ofs;
	if (!liblas::Create(ofs, las_file))
		return;
    
    int xoffset = int(point_cloud->at(0).x);
	int yoffset = int(point_cloud->at(0).y);
	int zoffset = int(point_cloud->at(0).z);
    liblas::Header header;
	header.SetVersionMajor(1);
	header.SetVersionMinor(2);
	header.SetOffset(xoffset, yoffset, zoffset);
	header.SetDataFormatId(liblas::ePointFormat3);
	header.SetPointRecordsCount(point_cloud->size());
	header.SetScale(0.0001, 0.0001, 0.0001);
	liblas::Writer writer(ofs, header);

    double Max_x = point_cloud->at(0).x, Min_x = point_cloud->at(0).x;
	double Max_y = point_cloud->at(0).y, Min_y = point_cloud->at(0).y;
	double Max_z = point_cloud->at(0).z, Min_z = point_cloud->at(0).z;
	// std::cout << "hello here!" << std::endl;
	for (int i = 0; i < point_cloud->size(); i++)
	{
		pcl::PointXYZ p = point_cloud->at(i);
 
		liblas::Point point(&header);
		point.SetRawX((p.x - xoffset) * 10000);
		point.SetRawY((p.y - yoffset) * 10000);
		point.SetRawZ((p.z - zoffset) * 10000);

		writer.WritePoint(point);
 
		if (p.x > Max_x)Max_x = p.x;
		if (p.y > Max_y)Max_y = p.y;
		if (p.z > Max_z)Max_z = p.z;
		if (p.x < Min_x)Min_x = p.x;
		if (p.y < Min_y)Min_y = p.y;
		if (p.z < Min_z)Min_z = p.z;
	}

	header.SetMax(Max_x, Max_y, Max_z);
	header.SetMin(Min_x, Min_y, Min_z);
	
	writer.SetHeader(header);
	writer.WriteHeader();
	ofs.close();

    std::cout << "write " << las_file << " (size: " << point_cloud->size() << " points) finish!" << std::endl;
}

// transfromation type
void matrix_to_pair(Eigen::Matrix4f &trans_matrix,
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &trans_pair)
{
    trans_pair.first = trans_matrix.block<3,1>(0,3).cast<double>();
    trans_pair.second = trans_matrix.block<3,3>(0,0).cast<double>();
}

void pair_to_matrix(Eigen::Matrix4f &trans_matrix,
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &trans_pair)
{
    trans_matrix.block<3,1>(0,3) = trans_pair.first.cast<float>();
    trans_matrix.block<3,3>(0,0) = trans_pair.second.cast<float>();
}

void point_to_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr &pclPoints, 
                        std::vector<Eigen::Vector3d> &vecPoints)
{
	int Num = pclPoints->size();
	
	for(int i=0; i<Num; i++)
	{
		Eigen::Vector3d p;
		p[0] = pclPoints->points[i].x;
		p[1] = pclPoints->points[i].y;
		p[2] = pclPoints->points[i].z;

		vecPoints.push_back(p);
	}
}

// down sample the point cloud, by voxel
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZ> &pl_feat, double voxel_size) 
{
	int pointsNum = pl_feat.size();
	if (voxel_size < 0.01) 
		return;

	// a container, include key and value
	std::unordered_map<UNI_VOXEL_LOC, M_POINT> voxel_map;
	// point cloud size
	uint plsize = pl_feat.size();
	// loop for all points, and sum the information in each grid
	for (uint i = 0; i < plsize; i++) {
	// current points
	pcl::PointXYZ &p_c = pl_feat[i];
	float loc_xyz[3];
	for (int j = 0; j < 3; j++) {
		loc_xyz[j] = p_c.data[j] / voxel_size;
		// <0, then loc-1, floor
		if (loc_xyz[j] < 0) {
		loc_xyz[j] -= 1.0;
		}
	}
	// generate the voxel location
	UNI_VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
						(int64_t)loc_xyz[2]);
	// 
	auto iter = voxel_map.find(position);
	if (iter != voxel_map.end()) {
		iter->second.xyz[0] += p_c.x;
		iter->second.xyz[1] += p_c.y;
		iter->second.xyz[2] += p_c.z;
		iter->second.count++;
	} else {
		M_POINT anp;
		anp.xyz[0] = p_c.x;
		anp.xyz[1] = p_c.y;
		anp.xyz[2] = p_c.z;
		anp.count = 1;
		voxel_map[position] = anp;
	}
	}

	// reset the ori point cloud data
	plsize = voxel_map.size();
	pl_feat.clear();
	pl_feat.resize(plsize);

	// downSample the point cloud, the centeroid points are selected 
	uint i = 0;
	for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) 
	{
		pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
		pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
		pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
		i++;
	}
	std::cout << "Input points num: " << pointsNum << ", downsample voxel size: " << voxel_size 
			  << ", downsampled points num: " << plsize << std::endl;
}

// calculate the accuracy of pose
void accur_evaluation(std::pair<Eigen::Vector3d, Eigen::Matrix3d> esti, Eigen::Affine3d truth,
					 std::pair<Eigen::Vector3d, Eigen::Vector3d> &errors)
{
	Eigen::Vector3d trans_error;
	Eigen::Vector3d rotation_error;
	
	// // translation error
	// trans_error = esti.first - truth.translation();
	// // rotation error
	// Eigen::Vector3d estiEuler=esti.second.eulerAngles(2,1,0);
	// Eigen::Vector3d truthEuler=truth.rotation().eulerAngles(2,1,0);
	// rotation_error = estiEuler - truthEuler;


	// trans the pose type into the pair vector and matrix
	std::pair<Eigen::Vector3d, Eigen::Matrix3d> truth_pose;
	truth_pose.first = truth.translation();
	truth_pose.second = truth.rotation();
	// calculate the error
	std::pair<Eigen::Vector3d, Eigen::Matrix3d> esti_inv = matrixInv(esti);
	std::pair<Eigen::Vector3d, Eigen::Matrix3d> esti_err = matrixMultip(esti_inv, truth_pose);
	// translation and rotation error
	trans_error = esti_err.first;
	rotation_error = esti_err.second.eulerAngles(2,1,0);

	for(int i=0; i<3; i++)
	{
		if(abs(rotation_error[i] - 2*M_PI) < 0.3)
			rotation_error[i] -= 2*M_PI;

		if(abs(rotation_error[i] + 2*M_PI) < 0.3)
			rotation_error[i] += 2*M_PI;
		
		if(abs(rotation_error[i] - M_PI) < 0.3)
			rotation_error[i] -= M_PI;

		if(abs(rotation_error[i] + M_PI) < 0.3)
			rotation_error[i] += M_PI;
	}
	
	// use the degree to evoluate the rotation error
	// rotation_error = rotation_error * (180/M_PI);
	
	// use the mrad to evoluate the rotation error
	rotation_error = rotation_error * 1000;
	
	errors.first = trans_error;
	errors.second = rotation_error;
	// std::cout << "trans: \n" << trans_error << "\nrotation: \n" << rotation_error << std::endl;
}
// write the accuracy result
void write_error(std::string filePath, std::pair<Eigen::Vector3d, Eigen::Vector3d> &errors)
{
	Eigen::Vector3d trans_error = errors.first;
	Eigen::Vector3d rotation_error = errors.second;

	std::ofstream outfile(filePath);
	if (!outfile.is_open()) 
	{
		std::cerr << "failed open this file!" << std::endl;
		return;
	}
	outfile << trans_error[0] << ", " << trans_error[1] << ", " << trans_error[2] << ", ";
	outfile << rotation_error[0] << ", " << rotation_error[1] << ", " << rotation_error[2] << std::endl;
	outfile.close();
}

// calculate the accuracy vector
void accur_evaluation_vec(std::vector<TLSPos> esti, std::vector<Eigen::Affine3d> truh, std::vector<PosError> &errors)
{
	std::pair<Eigen::Vector3d, Eigen::Vector3d> err;
	int ID;
	PosError pe;

	for(int i=0; i<esti.size(); i++)
	{
		// get the station ID
		ID = esti[i].ID;
		
		// get the estimated pose
		std::pair<Eigen::Vector3d, Eigen::Matrix3d> esti_pose;
		esti_pose.first = esti[i].t;
		esti_pose.second = esti[i].R;
		
		// evaluaion
		accur_evaluation(esti_pose, truh[ID], err);
		
		// get the ID and evaluation result
		pe.ID = ID;
		pe.error = err;
		errors.push_back(pe);
	}
}

// write the result vector
void write_error_vec(std::string filePath, std::vector<PosError> &errors)
{
	std::ofstream outfile(filePath);
	if (!outfile.is_open()) 
	{
		std::cerr << "failed open this file!" << std::endl;
		return;
	}
	
	for(int i=0; i<errors.size(); i++)
	{
		int id = errors[i].ID;
		Eigen::Vector3d trans_error = errors[i].error.first;
		Eigen::Vector3d rotation_error = errors[i].error.second;

		outfile << "Station: " << id << std::endl;
		outfile << trans_error[0] << ", " << trans_error[1] << ", " << trans_error[2] << ", ";
		outfile << rotation_error[0] << ", " << rotation_error[1] << ", " << rotation_error[2] << std::endl;
	}
	outfile.close();
}

void write_pose(std::string filePath, std::vector<TLSPos> poses)
{
	std::ofstream outfile(filePath);
	if (!outfile.is_open()) 
	{
		std::cerr << "failed open this file: " << filePath << std::endl;
		return;
	}
	for(int i=0; i<poses.size(); i++)
	{
		int id = poses[i].ID;
		Eigen::Matrix3d R = poses[i].R;
		Eigen::Vector3d t = poses[i].t;
		outfile << "Station: " << id << std::endl;
		outfile << t[0] << ", " << t[1] << ", " << t[2] << std::endl;
		outfile << R << std::endl << std::endl;
	}
	outfile.close();
}

void write_relative_pose(std::string filePath, std::pair<Eigen::Vector3d, Eigen::Matrix3d> poses)
{
	std::ofstream outfile(filePath);
	if (!outfile.is_open()) 
	{
		std::cerr << "failed open this file: " << filePath << std::endl;
		return;
	}
	outfile << "Trans: \n" << poses.first << std::endl;
	outfile << "Rot: \n" << poses.second << std::endl;
	
	outfile.close();
}

// get the absolut pose by relative pose between nodes
void RelaToAbs(std::vector<CandidateInfo> &candidates_vec, std::vector<TLSPos> &tlsVec)
{
    for(int num=0; num<2; num++)
	{
		// calculated the initial pos of each TLS station
		for(int i=0; i<candidates_vec.size(); i++)
		{
			TLSPos currPos;
			int candtlsID, tlsID = candidates_vec[i].currFrameID;
			// set the first station
			if(tlsID == 0)
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
	}
}

// get the absolut pose (DFS)
void AbsByDFS(int current_node, TLSPos &current_pose, std::vector<CandidateInfo> &candidates_vec, 
				std::vector<TLSPos> &tlsVec, std::unordered_set<int>& visited)
{
	visited.insert(current_node);
	tlsVec[current_node] = current_pose;
	for (int i=0; i<candidates_vec[current_node].candidateIDScore.size(); i++)
	{
		if (visited.find(candidates_vec[current_node].candidateIDScore[i].first) == visited.end())
		{
			int targetID = candidates_vec[current_node].candidateIDScore[i].first;
			TLSPos newPose;
			newPose.ID = targetID;
			newPose.isValued = true;
			newPose.R = current_pose.R * candidates_vec[current_node].relativePose[i].second.inverse();
			newPose.t = current_pose.t - 
				current_pose.R * candidates_vec[current_node].relativePose[i].second.inverse() * candidates_vec[current_node].relativePose[i].first;

			AbsByDFS(targetID, newPose, candidates_vec, tlsVec, visited);
		}
	}
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d> matrixInv(std::pair<Eigen::Vector3d, Eigen::Matrix3d> m1)
{
	std::pair<Eigen::Vector3d, Eigen::Matrix3d> res;
	
	res.second = m1.second.inverse();
	res.first = -m1.second.inverse() * m1.first;

	return res;
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d> matrixMultip(std::pair<Eigen::Vector3d, Eigen::Matrix3d> m1, 
                                                        std::pair<Eigen::Vector3d, Eigen::Matrix3d> m2)
{
	std::pair<Eigen::Vector3d, Eigen::Matrix3d> res;

	res.second = m1.second * m2.second;
	res.first = m1.second*m2.first + m1.first;

	return res;
}

