#ifndef _STATE_H_
#define _STATE_H_

#include <pose_estimate/my_include.h>

#define STATE_NUM 12

namespace slam_mono
{
struct ImuState
{
    
    // state var
    Eigen::Quaterniond q;      // attitude ([q1, q2, q3, q0(scalar)]) >> from world to imu
    Eigen::Vector3d p;         // positon ( IMU centered )
    Eigen::Vector3d v;         // velocity

    Eigen::Vector3d bw;        // gyro bias : x, y, z
    Eigen::Vector3d ba;        // accel bias : x, y, z
    
    Eigen::Quaterniond q_wv;   // visual-world attitude drift : from vision to world
    Eigen::Quaterniond q_ci;   // camer-imu attitude calibration : from imu to camera
    Eigen::Vector3d p_ci;      // camera-imu position calibration : camera's origin in imu frame

    Eigen::Matrix<double, STATE_NUM, STATE_NUM> state_cov;
    Eigen::Matrix<double, 4, 4> R;
    Eigen::Matrix<double, STATE_NUM, 1> xCorrect;
};

struct Parameter
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // process noise
    double gyro_noise;
    double acc_noise;
    double gyro_bias_noise;
    double acc_bias_noise;
    
    // measurement noise
    double camear_q_noise;

    // Gravity in world
    Eigen::Vector3d gravity;

    // bias of gyro
    Eigen::Vector3d gyro_bias;

    bool fixed_bias;
    bool fixed_calibr;

    Eigen::Matrix<double, 6, 6> continues_noise_cov;

    double time;
};
} /*namespace end*/


#endif