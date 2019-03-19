#ifndef _MY_INCLUDE_H_
#define _MY_INCLUDE_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <feature_tracker/FeatureTrackerResult.h>
#include <feature_tracker/CameraTrackerResult.h>
#include <pose_estimate/PoseEstimateResult.h>

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace cv;


#endif