#ifndef _G2O_TYPE_H_
#define _G2O_TYPE_H_

#include <pose_est_new/my_includes.h>

namespace slam_mono
{
class EdgeProjectXYZStoreo : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read( std::istream& in ){}
    virtual bool write( std::ostream& out) const {}
    
};

}



#endif