#include <pose_est_new/pose_est_new.h>
#include <pose_est_new/CameraState.h>
#include <pose_est_new/DataCollectionForSim.h>

namespace slam_mono
{

PoseEst::PoseEst(void): isSensorCalibr(false), camera(new CameraPose)
{
    n.param<string>("imu_topics", IMU_TOPIC, string("/mynteye/imu/data_raw_processed"));
    n.param<string>("pub_feature_topic", FEATURE_TOPIC, string("/feature_tracker/points"));
    
    T_left2imu << 0.99996651999999997,  0.00430873000000000,  0.00695718000000000, -0.04777362000000000108,
                  0.00434878000000000, -0.99997400999999997, -0.00575128000000000, -0.002237309999999999910,
                  0.00693222000000000,  0.00578135000000000, -0.99995926000000002, -0.001600710000000000080,
                  0.00000000000000000,  0.00000000000000000,  0.00000000000000000,  1.000000000000000000;
    cout << "T_left2imu: " << endl << T_left2imu << endl;
    
    featureSub = n.subscribe(FEATURE_TOPIC, 1, &PoseEst::Feature_Callback, this);
    imuSub = n.subscribe(IMU_TOPIC, 1, &PoseEst::Imu_Callback, this);

    cameraStatePub = n.advertise<pose_est_new::CameraState>("/slam_mono/cameraState", 1);
    dataCollectionForSimPub = n.advertise<pose_est_new::DataCollectionForSim>("/slam_mono/dataCollectionForSim",1);
    ROS_INFO("parameters load success!");
}

PoseEst::~PoseEst()
{

}

void PoseEst::Imu_Callback(const sensor_msgs::ImuConstPtr& msg)
{
    pose_est_new::DataCollectionForSim msgDataCollectionForSim;
    msgDataCollectionForSim.angular_velocity_imu = msg->angular_velocity;
    msgDataCollectionForSim.linear_acceleration_imu = msg->linear_acceleration;

    msgDataCollectionForSim.header.stamp = msg->header.stamp;
    if(cameraStateUpdate==1)
    {
        cameraStateUpdate = 0;
        msgDataCollectionForSim.orientation_cam.w = camera->q.w();
        msgDataCollectionForSim.orientation_cam.x = camera->q.x();
        msgDataCollectionForSim.orientation_cam.y = camera->q.y();
        msgDataCollectionForSim.orientation_cam.z = camera->q.z();
        msgDataCollectionForSim.pos_cam.x = camera->p(0);
        msgDataCollectionForSim.pos_cam.y = camera->p(1);
        msgDataCollectionForSim.pos_cam.z = camera->p(2);
        msgDataCollectionForSim.updateCameraState = 1;
    }
    else
    {
        msgDataCollectionForSim.updateCameraState = 0;
        msgDataCollectionForSim.orientation_cam.w = 0.0;
        msgDataCollectionForSim.orientation_cam.x = 0.0;
        msgDataCollectionForSim.orientation_cam.y = 0.0;
        msgDataCollectionForSim.orientation_cam.z = 0.0;
        msgDataCollectionForSim.pos_cam.x = 0.0;
        msgDataCollectionForSim.pos_cam.y = 0.0;
        msgDataCollectionForSim.pos_cam.z = 0.0;
    }
    dataCollectionForSimPub.publish(msgDataCollectionForSim);
}

void PoseEst::Feature_Callback(const feature_tracker::CameraTrackerResultConstPtr& msg)
{
    double timeBegin = ros::Time::now().toNSec();
    Clear_Points(featuresCurr);

    Put_Feature_into_Vector(msg);

    if( !featuresRef.pts3dMap.empty())
    {
        Find_Feature_Matches();
        camera->Bundle_Adjustment(ptsRefMatched, ptsCurrMatched);
        //
        {
            pose_est_new::CameraState msgCameraState;
            msgCameraState.q0 = camera->q.w();
            msgCameraState.q1 = camera->q.x();
            msgCameraState.q2 = camera->q.y();
            msgCameraState.q3 = camera->q.z();
            msgCameraState.x = camera->p(0);
            msgCameraState.y = camera->p(1);
            msgCameraState.z = camera->p(2);
            cameraStateUpdate = 1;
            cameraStatePub.publish(msgCameraState);
        }
    }

    featuresRef = featuresCurr;
    double timeEnd = ros::Time::now().toNSec();
    ROS_INFO_STREAM("time cost: " << (timeEnd - timeBegin) << " ns");
}

void PoseEst::Put_Feature_into_Vector(const feature_tracker::CameraTrackerResultConstPtr& msg)
{
    featuresCurr.header = msg->header;
    for (int i = 0; i < msg->num_of_features; i++)
    {
        if(msg->features[i].z >= 0)  // 深度值可用
        {
            // 将相机系下的点投影到imu系
            Vector4d pts0_4d(msg->features[i].x, msg->features[i].y, msg->features[i].z, 1.0);
            Vector4d pts1_4d = T_left2imu * pts0_4d;
            featuresCurr.id.push_back(msg->features[i].id);
            featuresCurr.cnt.push_back(msg->features[i].cnt);
        
            Vector3d ptsTemp(pts1_4d(0), pts1_4d(1), pts1_4d(2));
            
            featuresCurr.pts3d.push_back(ptsTemp);
            featuresCurr.pts3dMap.insert(make_pair(msg->features[i].id, ptsTemp));
        }
    }
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
