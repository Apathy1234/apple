#ifndef _POSE_ESTIMATE_H_
#define _POSE_ESTIMATE_H_

#include <pose_estimate/my_include.h>
#include <pose_estimate/g2o_edge.h>

namespace slam_mono
{

class PoseEstimate
{
public:
    typedef shared_ptr<PoseEstimate> ptr;
    PoseEstimate(void);
    ~PoseEstimate(void);
    void Feature_Callback(const feature_tracker::CameraTrackerResultPtr& pts);
    void Imu_Callback(const sensor_msgs::ImuConstPtr& imuMsg);
private:
    typedef long long int LONGTYPE;

    // ros相关
    string FEATURE_TOPICS;
    string IMU_TOPICS;
    ros::NodeHandle n;
    ros::Subscriber featureSub; 
    ros::Subscriber imuSub;
    ros::Publisher posePub;
    
    // imu相关
    bool isSensorCalib;
    vector<sensor_msgs::Imu> imuMsgBuffer;
    
    // 特征点相关
    struct Features
    {
        std_msgs::Header header;
        vector<LONGTYPE> id;
        vector<LONGTYPE> cnt;
        vector<Point3f> pts3d;
        map<LONGTYPE, Point3f> pts3dMap;
    };
    
    Features featuresRef;
    Features featuresCurr;
    vector<Point3f> ptsRefMatched;
    vector<Point3f> ptsCurrMatched;
    
    // 姿态相关
    Mat T_left2imu;
    Matx33d r_left2imu;
    Vec3d t_left2imu;
    Eigen::Matrix3d eigenR;
    Mat estR;
    Vec3d estT;
    Mat R;
    Mat T;
    Eigen::Quaterniond qDet;
    Eigen::Vector3d tDet;


    uint64 currentTime;
    uint64 lastTime;
    void Clear_Points(Features& fet);
    void Find_Feature_Matches(void);
    void Bundle_Adjustment(const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t);
    void Predict_With_IMU(void);
};

}


#endif