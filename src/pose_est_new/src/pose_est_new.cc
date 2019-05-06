#include <pose_est_new/pose_est_new.h>

namespace slam_mono
{

PoseEst::PoseEst(void): isSensorCalibr(false), camera(new CameraPose)
{
    n.param<string>("imu_topics", IMU_TOPIC, string("/mynteye/imu/data_raw_processed"));
    n.param<string>("pub_feature_topic", FEATURE_TOPIC, string("/feature_tracker/points"));

    featureSub = n.subscribe(FEATURE_TOPIC, 1, &PoseEst::Feature_Callback, this);
    imuSub = n.subscribe(IMU_TOPIC, 50, &PoseEst::Imu_Callback, this);
    ROS_INFO("parameters load success!");
}

PoseEst::~PoseEst()
{

}

void PoseEst::Imu_Callback(const sensor_msgs::ImuConstPtr& msg)
{

}

void PoseEst::Feature_Callback(const feature_tracker::CameraTrackerResultConstPtr& msg)
{
    double timeBegin = ros::Time::now().toNSec();
    Clear_Points(featuresCurr);

    // put the feature into vector
    featuresCurr.header = msg->header;
    for (int i = 0; i < msg->num_of_features; i++)
    {
        if(msg->features[i].z >= 0)  // 深度值可用
        {
            featuresCurr.id.push_back(msg->features[i].id);
            featuresCurr.cnt.push_back(msg->features[i].cnt);
        
            Vector3d ptsTemp(msg->features[i].x, msg->features[i].y, msg->features[i].z);
            featuresCurr.pts3d.push_back(ptsTemp);
            featuresCurr.pts3dMap.insert(make_pair(msg->features[i].id, ptsTemp));
        }
    }
    if( !featuresRef.pts3dMap.empty())
    {
        Find_Feature_Matches();
        camera->Bundle_Adjustment(ptsRefMatched, ptsCurrMatched);
    
    }

    featuresRef = featuresCurr;
    double timeEnd = ros::Time::now().toNSec();
    ROS_INFO_STREAM("time cost: " << (timeEnd - timeBegin) << " ns");
}

void PoseEst::Clear_Points(FeatureState& feature)
{
    feature.id.clear();
    feature.cnt.clear();
    feature.pts3d.clear();
    feature.pts3dMap.clear();
}

void PoseEst::Find_Feature_Matches(void)
{
    ptsRefMatched.clear();
    ptsCurrMatched.clear();
    for(int i = 0; i < featuresCurr.id.size(); i++)
    {
        if(featuresCurr.cnt[i] != 1) // 并非只追踪了一次
        {
            map<FeatureIDType, Vector3d>::iterator it;
            it = featuresRef.pts3dMap.find(featuresCurr.id[i]);
            if(it != featuresRef.pts3dMap.end())
            {
                ptsRefMatched.push_back(it->second);
                ptsCurrMatched.push_back(featuresCurr.pts3d[i]);
            }
        }
    }
    ROS_INFO_STREAM("the number of matching points found is: " << ptsRefMatched.size());
}

}
