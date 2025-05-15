#ifndef _DST_H_Included_
#define _DST_H_Included_
#include "DST.h"
#endif

// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

void GTSAMResToPose(gtsam::Values &result, std::vector<TLSPos> &optiTLSVec);

void GTSAMOptimization(std::vector<TLSPos> tlsVec, std::vector<CandidateInfo> candidates_vec, gtsam::Values &result, std::pair<double, double> var);
