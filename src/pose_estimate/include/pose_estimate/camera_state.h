#ifndef _CAMERA_STATE_H_
#define _CAMERA_STATE_H_

#include <pose_estimate/my_include.h>

namespace slam_mono
{
typedef long long int StateIDType;
struct CameraState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateIDType id;

    double time;

    Eigen::Quaterniond orientation;

    Eigen::Vector3d position;

    CameraState(): id(0), time(0), orientation(Eigen::Quaterniond(1, 0, 0, 0)), position(Eigen::Vector3d::Zero()) {}

};

/*namespace end*/
}

#endif
