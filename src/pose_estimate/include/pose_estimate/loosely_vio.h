#ifndef _LOOSELY_VIO_H_
#define _LOOSELY_VIO_H_

#include <pose_estimate/my_include.h>
#include <pose_estimate/g2o_edge.h>
#include <pose_estimate/camera_state.h>
#include <pose_estimate/feature.h>
#include <pose_estimate/math_utils.hpp>

namespace slam_mono
{

class Loosely_vio
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    typedef shared_ptr<Loosely_vio> ptr;
    Loosely_vio(void);
    ~Loosely_vio(void);
    void Feature_Callback(const feature_tracker::CameraTrackerResultPtr& pts);
private:
    
    // ros相关
    string FEATURE_TOPICS;
    string IMU_TOPICS;
    ros::NodeHandle n;
    ros::Subscriber featureSub; 
    ros::Subscriber imuSub;
    ros::Publisher posePub;

    // imu相关
    bool isSensorCalibr;
    vector<sensor_msgs::Imu> imuMsgBuffer;

    // 局部地图相关
    MapServer mapServer;
    map<FeatureIDType, int> pointMatchedMap;

    // 相机姿态相关
    CameraState cameraState;

    // 特征点相关
    vector<FeatureIDType> matchedFeatureID;    
    vector<Eigen::Vector3d> ptsMapMatched;
    vector<Eigen::Vector3d> ptsCurrMatched;
    float matchLessRatio;

    //　相关标志位
    bool isFirstData;
    double trackRate;

    void Add_Feature_Points(const feature_tracker::CameraTrackerResultPtr& pts);
    void Find_Feature_Matched(const feature_tracker::CameraTrackerResultPtr& pts);
    void Remove_Feature_Points(void);
    void Bundle_Adjustment(void);
    Eigen::Vector3d Calculate_World_Point(const Eigen::Vector3d pts);
    
};

}



#endif