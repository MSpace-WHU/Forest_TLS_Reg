#include "stdio.h"
#include <iostream>
#include<fstream>
#include <stack>
#include <unordered_set>
#include <tuple>

#include <ctime>
#include <string>
#include <cstdlib>
#include <chrono>

#include "liblas/liblas.hpp"
#include "liblas/point.hpp"
#include "boost/regex.hpp"

// OpenCV
#include "opencv2/core.hpp"

#ifndef _DST_H_Included_
#define _DST_H_Included_
#include "../include/dst/DST.h"
#endif


#define INSIGNIFICANCE -99999
#define INFINITE		99999
#define INFINITESIMAL 0.00000001


#define MAX(A,B) ((A) >= (B) ? (A) : (B))
#define MIN(A,B) ((A) <= (B) ? (A) : (B))


// Associated functions
bool New_Int(int **pnArray, long lEleNum);
bool New_Char(char **pcArray, long lEleNum);
bool New_Long(long **plArray, long lEleNum);
bool New_Double(double **pdArray, long lEleNum);
bool New_Bool(bool **pbArray, long lEleNum);

bool Del_Int(int **pnArray);
bool Del_Char(char **pcArray);
bool Del_Long(long **plArray);
bool Del_Double(double **pdArray);
bool Del_Bool(bool **pbArray);

void SetVal_Int(int* pnArray, long lEleNum, int nVal);
void SetVal_Long(long *plArray, long lEleNum, long lVal);
void SetVal_Double(double *pdArray, long lEleNum, double dVal);
void SetVal_Bool(bool *pbArray, long lEleNum, bool bVal);

bool Val_Equ(const int bVal_1, const int bVal_2);
bool Val_Equ(const double bVal_1, const double bVal_2);
bool Val_Equ(const long lVal_1, const long lVal_2);
bool Val_Equ(const char& cVal_1, const char& cVal_2);
// int round(double x);

long ArrayMax(const double *pdIn, const long lEleNum);
long ArrayMax(const long *plIn, const long lEleNum);
long ArrayMax(const int *pnIn, const long lEle_Num);

// read the parameters from yaml file 
void ReadParas(const std::string& file_path, ConfigSetting &config_setting);

// split the line into string
std::vector<std::string> split(std::string str,std::string s);

// read the trans file from LeiCa RTC360
std::vector<Eigen::Affine3d> readTLSTrans(const std::string tansFile);

// read the trans file of Tongji Trees dataset
void readTongjiTrans(const std::string& filename, 
                     std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& effectivenessData,
                     std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& robustnessData,
                     std::vector<std::tuple<int, int, Eigen::Matrix4d>>& transformations);


// read transformation from FGI data
void readFGITrans(const std::string filename, std::pair<Eigen::Vector3d, Eigen::Matrix3d>& trans);

// read and write the las data, stored by pointXYZ
void readTLSData(const std::string lasFile, pcl::PointCloud<pcl::PointXYZ>::Ptr &tlsPC);
void writeLas(std::string las_file, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

void matrix_to_pair(Eigen::Matrix4f &trans_matrix,
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &trans_pair);
void pair_to_matrix(Eigen::Matrix4f &trans_matrix,
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &trans_pair);

void point_to_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr &pclPoints, 
                        std::vector<Eigen::Vector3d> &vecPoints);

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZ> &pl_feat,
                         double voxel_size);

// accuracy evalution and write, for each pose
void accur_evaluation(std::pair<Eigen::Vector3d, Eigen::Matrix3d> esti, Eigen::Affine3d truth,
					 std::pair<Eigen::Vector3d, Eigen::Vector3d> &errors);
void write_error(std::string filePath, std::pair<Eigen::Vector3d, Eigen::Vector3d> &errors);

// accuracy evalution and write, for poses vector
void accur_evaluation_vec(std::vector<TLSPos> esti, std::vector<Eigen::Affine3d> truh, std::vector<PosError> &errors);

void write_error_vec(std::string filePath, std::vector<PosError> &errors);

// write the pose data
void write_pose(std::string filePath, std::vector<TLSPos> poses);

// write the rlative pose between two station
void write_relative_pose(std::string filePath, std::pair<Eigen::Vector3d, Eigen::Matrix3d> poses);

void RelaToAbs(std::vector<CandidateInfo> &candidates_vec, std::vector<TLSPos> &tlsVec);

void AbsByDFS(int current_node, TLSPos &current_pose, std::vector<CandidateInfo> &candidates_vec, 
				std::vector<TLSPos> &tlsVec, std::unordered_set<int>& visited);

/***the simple operations of matrix***/
// inv
std::pair<Eigen::Vector3d, Eigen::Matrix3d> matrixInv(std::pair<Eigen::Vector3d, Eigen::Matrix3d> m1);
// multiple
std::pair<Eigen::Vector3d, Eigen::Matrix3d> matrixMultip(std::pair<Eigen::Vector3d, Eigen::Matrix3d> m1, 
                                                        std::pair<Eigen::Vector3d, Eigen::Matrix3d> m2);


//============the following are UBUNTU/LINUX ONLY terminal color codes.==========
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
//============the following are UBUNTU/LINUX ONLY terminal color codes.==========

class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    TicToc( bool _disp )
    {
        disp_ = _disp;
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    void toc( std::string _about_task )
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double elapsed_ms = elapsed_seconds.count() * 1000;

        if( disp_ )
        {
          std::cout.precision(3); // 10 for sec, 3 for ms 
          std::cout << _about_task << ": " << elapsed_ms << " msec." << std::endl;
        }
    }

private:  
    std::chrono::time_point<std::chrono::system_clock> start, end;
    bool disp_ = false;
};
