#ifndef _FEATURE_STATE_H_
#define _FEATURE_STATE_H_

#include <pose_est_new/my_includes.h>

namespace slam_mono
{
typedef long long int FeatureIDType;
struct FeatureState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std_msgs::Header header;

    vector<FeatureIDType> id;
    vector<FeatureIDType> cnt;
    vector<Vector3d> pts3d;
    map<FeatureIDType, Vector3d> pts3dMap;

};


}





#endif