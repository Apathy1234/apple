#ifndef _FEATURE_H_
#define _FEATURE_H_

#include <pose_estimate/my_include.h>

namespace slam_mono
{

typedef long long int FeatureIDType;

struct FeatureState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d position;
    long long int keepTimes;   // the times that keep the feature
    long long int matchTimes;  // the times that match the feature

    FeatureState(): position(Eigen::Vector3d::Zero()), matchTimes(1), keepTimes(1){}

    FeatureState(const Eigen::Vector3d new_position): position(new_position), matchTimes(1), keepTimes(1){}
        
};


typedef map<FeatureIDType, FeatureState, less<FeatureIDType>, 
Eigen::aligned_allocator<
pair<const FeatureIDType, FeatureState> > > MapServer;
/*namespace end*/
}



#endif