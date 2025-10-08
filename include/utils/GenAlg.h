#include <Eigen/Dense>
#include<opencv2/core/eigen.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>

bool Profile(const double* pdValIn, const long lEleNum, const double dDTh, const int nStep, int* pnGrpID);
bool GrpInfo(const int* pnGrpID, const long lEleNum, const long lGrpEleNumTh, long* plGrpNum, GrpStaic** pGrpEle);
bool Grp_ID_Ord(int* pnGrp_ID, const long lEle_Num, const int nGrp_Ele_Min);
bool Ele_Num_Max(int* pnGrp_ID, const long lEle_Num, long* plGrp_ID_Max, long* plGrp_Ele_Num_Max, long* plGrp_Ele_Sta_End);

bool rowSeg(double* &dataArray,
            int Horizon_SCAN, int N_SCAN,
            double dDif_Th, int nSch_Stp, int lGrp_Ele_Min,
            cv::Mat &pointTypeMat);

bool colSeg(double* &dataArray,
            int Horizon_SCAN, int N_SCAN,
            double dDif_Th, int nSch_Stp, int lGrp_Ele_Min,
            cv::Mat &pointTypeMat);

void segPoints(cv::Mat &pointMat,
                int Horizon_SCAN, int N_SCAN,
                cv::Mat &pointTypeMat,
                ConfigSetting config_setting);