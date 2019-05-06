#ifndef _CAMERA_POSE_H_
#define _CAMERA_POSE_H_

#include <pose_est_new/my_includes.h>
#include <pose_est_new/g2o_type.h>

namespace slam_mono
{

typedef long long int StateIDType;

class CameraPose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef shared_ptr<CameraPose> ptr;
    CameraPose();
    virtual ~CameraPose();
    void Bundle_Adjustment(const vector<Vector3d>& pts1, const vector<Vector3d>& pts2);
    StateIDType id;
    double time;
    Quaterniond q;
    Vector3d p;
};

}



#endif