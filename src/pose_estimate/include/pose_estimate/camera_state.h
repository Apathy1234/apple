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

    Eigen::Vector4d orientation;

    Eigen::Vector3d position;

    CameraState(): id(0), time(0), orientation(Eigen::Vector4d(0, 0, 0, 1)), position(Eigen::Vector3d::Zero()) {}

};

/*namespace end*/
}

#endif
