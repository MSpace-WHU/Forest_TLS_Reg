#ifndef _GTSAMOPTI_H_Included_
#define _GTSAMOPTI_H_Included_
#include "../include/gtsam_opti/gtsamOpti.h"
#endif

// gtsam result to the tls pose
void GTSAMResToPose(gtsam::Values &result, std::vector<TLSPos> &optiTLSVec)
{
	for(int i=0; i<result.size(); i++)
    {
        TLSPos curr_transform;
        curr_transform.ID = i;
        curr_transform.R = result.at<gtsam::Pose3>(i).rotation().matrix();
        curr_transform.t = result.at<gtsam::Pose3>(i).translation();
        curr_transform.isValued = true;
        
        optiTLSVec.push_back(curr_transform);
    }
}

// gtsam optimization
void GTSAMOptimization(std::vector<TLSPos> tlsVec, std::vector<CandidateInfo> candidates_vec, gtsam::Values &result, std::pair<double, double> var)
{
	// set the initial value of each graph node
	gtsam::Values initial;
    for(int i=0; i<tlsVec.size(); i++)
    {
        if(tlsVec[i].isValued)
        {
            std::cout << "Station: " << tlsVec[i].ID << std::endl;
            std::cout << "R: " << std::endl;
            std::cout << tlsVec[i].R << std::endl;
            std::cout << "t: " << std::endl;
            std::cout << tlsVec[i].t << std::endl;

            gtsam::Rot3 initR(tlsVec[i].R);
            gtsam::Point3 initT(tlsVec[i].t);
            initial.insert(tlsVec[i].ID, gtsam::Pose3(initR, initT));
            // initial.insert(tlsVec[i].ID, Pose3(gtsam::Rot3::identity(), gtsam::Point3::Zero()));
        }
    }
	// new a gtsam graph
	gtsam::NonlinearFactorGraph gtSAMGraph;
    gtsam::Rot3 R = gtsam::Rot3::identity();
    gtsam::Point3 t = gtsam::Point3::Zero();
    gtsam::Pose3 priorPos(R, t);
	// rad*rad, meter*meter
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
    // gtSAMGraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, priorPos, priorNoise));
    // NO noise
    gtSAMGraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, priorPos));

    double v_rad = var.first;
    double v_met = var.second;
    // add the between constraints
    for(int i=0; i<candidates_vec.size(); i++)
    {
        int currID = candidates_vec[i].currFrameID;
        for(int j=0; j<candidates_vec[i].candidateIDScore.size(); j++)
        {
            int candID = candidates_vec[i].candidateIDScore[j].first;
            gtsam::Point3 diffT(candidates_vec[i].relativePose[j].first);
            gtsam::Rot3 diffR(candidates_vec[i].relativePose[j].second);

            gtsam::Pose3 odometry(diffR, diffT);

            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << v_rad, v_rad, v_rad, v_met, v_met, v_met).finished());
            gtSAMGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(candID, currID, odometry, odometryNoise));

            // // remove the cross error, such Plot-4 in Tongji-Tree Dataset
            // gtsam::Vector sigma(6);
            // sigma << sqrt(v_rad), sqrt(v_rad), sqrt(v_rad), sqrt(v_met), sqrt(v_met), sqrt(v_met);
            // auto noiseModel = gtsam::noiseModel::Robust::Create(
            //                     gtsam::noiseModel::mEstimator::Huber::Create(1), // Huber
            //                     gtsam::noiseModel::Diagonal::Sigmas(sigma));
            // gtSAMGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(candID, currID, odometry, noiseModel));
        }
    }
    gtSAMGraph.print();
	
	// set the gtsam parameters
	gtsam::LevenbergMarquardtParams params_lm;//构建LM算法参数类(相当于g2o options)
    params_lm.setVerbosity("ERROR");//设置输出信息
    params_lm.setMaxIterations(20);//最大迭代次数
    params_lm.setLinearSolverType("MULTIFRONTAL_QR");//分解算法
    
	// LM optimizer
	gtsam::LevenbergMarquardtOptimizer optimizer_LM( gtSAMGraph, initial, params_lm );//构建下降算法(图,初值,参数)
    result = optimizer_LM.optimize();
    
	result.print();
}