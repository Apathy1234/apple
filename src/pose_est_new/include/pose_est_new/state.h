#ifndef _STATE_H_
#define _STATE_H_

#include <pose_est_new/my_includes.h>

#define STATE_NUM 16
#define MEASURE_NUM 7

namespace slam_mono
{
   struct ImuState
   {
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW
       // state var
       Quaterniond q;   // attitude ([q1, q2, q3, q0(scalar)]) >> from world to imu
       Vector3d p;      // position (IMU centered)
       Vector3d v;      // velocity

       Vector3d bw;     // gyro bias : x, y, z
       Vector3d ba;     // accel bias : x, y, z

       Matrix<double, STATE_NUM, STATE_NUM> state_cov;
       Matrix<double, STATE_NUM, STATE_NUM> Q;
       Matrix<double, 3, 3> R_accel;
       Matrix<double, MEASURE_NUM, MEASURE_NUM> R_cam;
   };

    struct Parameter
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // state_cov noise
        double quat_uncertainty;
        double pose_uncertainty_m;
        double vel_uncertainty_mps;
        double gyro_bias_noise;
        double acc_bias_noise;

        // process noise
        double quat_process_noise;
        double pos_process_noise_m;
        double vel_process_noise_mps;
        double gyro_process_noise_rps;
        double acc_process_noise_mps2;

        // measurement noise
        double cam_trans_means_m;
        double cam_pose_means;
        double imu_acc_means_mps2;

        // measurement noise
        double camera_q_noise;
        double camera_p_noise;

        // bias of gyro
        Vector3d gyro_bias;

        //gravity in world
        Vector3d gravity;

        double time;
    };
    
/*
 *  @brief Create a skew-symmetric matrix from a 3-element vector.
 *  @note Performs the operation:
 *  w   ->  [  0 -w3  w2]
 *          [ w3   0 -w1]
 *          [-w2  w1   0]
 */
    inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w)
    {
        Eigen::Matrix3d w_sym;

        w_sym(0, 0) =     0;
        w_sym(0, 1) = -w(2);
        w_sym(0, 2) =  w(1);
        w_sym(1, 0) =  w(2);
        w_sym(1, 1) =     0;
        w_sym(1, 2) = -w(0);
        w_sym(2, 0) = -w(1);
        w_sym(2, 1) =  w(0);
        w_sym(2, 2) =     0;
        return w_sym;
    }
    
} // namespace slam_mono



#endif