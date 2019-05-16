#ifndef _POSE_EST_NEW_H_
#define _POSE_EST_NEW_H_

#include <pose_est_new/my_includes.h>
#include <pose_est_new/feature_state.h>
#include <pose_est_new/camera_pose.h>
#include <pose_est_new/CamreaState.h>
#include <pose_est_new/state.h>

namespace slam_mono
{

class PoseEst
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef shared_ptr<PoseEst> ptr;
    PoseEst();
    virtual ~PoseEst();
    void Imu_Callback(const sensor_msgs::ImuConstPtr& msg);
    void Feature_Callback(const feature_tracker::CameraTrackerResultConstPtr& msg);
private:
    // ros
    string FEATURE_TOPIC;
    string IMU_TOPIC;
    ros::NodeHandle n;
    ros::Subscriber featureSub;
    ros::Subscriber imuSub;
    ros::Publisher cameraStatePub;  int cameraStateUpdate = 0;
    ros::Publisher dataCollectionForSimPub;
    
    // imu
    vector<sensor_msgs::Imu> imuMsgBuffer;

    // feature
    FeatureState featuresRef;
    FeatureState featuresCurr;
    vector<Vector3d> ptsRefMatched;
    vector<Vector3d> ptsCurrMatched;

    // camera
    CameraPose::ptr camera;

    // rotation
    Matrix4d T_left2imu;
    Matrix3d T_sensor;

    // flag
    bool isFirstImage;
    bool isSensorCalibr;

    // filter
    Parameter params;
    ImuState state_delay;

    void Init_Sensor(void);
    void Process_Model(const double& time, Vector3d& gyro_mes,Vector3d& acc_mes);
    void Deal_with_IMU(const double& timeBond);
    void Put_Feature_into_Vector(const feature_tracker::CameraTrackerResultConstPtr& msg);
    void Clear_Points(FeatureState& feature);
    void Find_Feature_Matches(void);
};


}



#endif