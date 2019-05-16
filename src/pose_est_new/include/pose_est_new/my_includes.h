#ifndef _MY_INCLUDES_H
#define _MY_INCLUDES_H

#include <ros/ros.h>
#include <feature_tracker/FeatureTrackerResult.h>
#include <feature_tracker/CameraTrackerResult.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>

#include <map>
#include <string>
#include <vector>
#include <cmath>

#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <g2o/types/slam3d/se3quat.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/factory.h>



using namespace std;
using namespace Eigen;


#endif