#include <pose_est_new/pose_est_new.h>
#include <pose_est_new/CameraState.h>
#include <pose_est_new/DataCollectionForSim.h>

namespace slam_mono
{

PoseEst::PoseEst(void): isSensorCalibr(false), camera(new CameraPose), isFirstImage(true)
{
    n.param<string>("imu_topics", IMU_TOPIC, string("/mynteye/imu/data_raw_processed"));
    n.param<string>("pub_feature_topic", FEATURE_TOPIC, string("/feature_tracker/points"));
    // // mynteye
    // T_left2imu << 0.99996651999999997,  0.00430873000000000,  0.00695718000000000, -0.04777362000000000108,
    //               0.00434878000000000, -0.99997400999999997, -0.00575128000000000, -0.002237309999999999910,
    //               0.00693222000000000,  0.00578135000000000, -0.99995926000000002, -0.001600710000000000080,
    //               0.00000000000000000,  0.00000000000000000,  0.00000000000000000,  1.000000000000000000;
    // eurco
    T_left2imu << 0.014865542981800,  -0.999880929698000,   0.004140296794220,  -0.021640145497500,
                  0.999557249008000,   0.014967213324700,   0.025715529948000,  -0.064676986768000,
                 -0.025774436697400,   0.003756188357970,   0.999660727178000,   0.009810730589490,
                  0.000000000000000,   0.000000000000000,   0.000000000000000,   1.000000000000000;
                  
    cout << "T_left2imu: " << endl << T_left2imu << endl;
    
    featureSub = n.subscribe(FEATURE_TOPIC, 1, &PoseEst::Feature_Callback, this);
    imuSub = n.subscribe(IMU_TOPIC, 50, &PoseEst::Imu_Callback, this);

    cameraStatePub = n.advertise<pose_est_new::CameraState>("/slam_mono/cameraState", 1);
    dataCollectionForSimPub = n.advertise<pose_est_new::DataCollectionForSim>("/slam_mono/dataCollectionForSim",50);
    ROS_INFO("parameters load success!");
}

PoseEst::~PoseEst()
{

}

void PoseEst::Imu_Callback(const sensor_msgs::ImuConstPtr& msg)
{
    imuMsgBuffer.push_back(*msg);
}

void PoseEst::Feature_Callback(const feature_tracker::CameraTrackerResultConstPtr& msg)
{
    double timeBegin = ros::Time::now().toNSec();
    // 清除当前特征点信息
    Clear_Points(featuresCurr);
    // 将当前特征点放入容器
    Put_Feature_into_Vector(msg);

    if (isFirstImage)
    {
        isFirstImage = false;
        firstImageTime = msg->header.stamp.toSec();
    }
    else
    {
        Find_Feature_Matches();
        camera->Bundle_Adjustment(ptsRefMatched, ptsCurrMatched);
        Deal_with_IMU(msg->header.stamp.toSec());
    }
    
    featuresRef = featuresCurr;

    double timeEnd = ros::Time::now().toNSec();
    ROS_INFO_STREAM("time cost: " << (timeEnd - timeBegin) << " ns");
}

void PoseEst::Deal_with_IMU(const double& timeBond)
{
    int imu_msg_used_cnt = 0;
    
    for( vector<sensor_msgs::Imu>::iterator imu = imuMsgBuffer.begin(); imu != imuMsgBuffer.end(); imu++)
    {
        vector<sensor_msgs::Imu>::iterator imu_next = imu + 1;
        
        double imuTime = imu->header.stamp.toSec();
        if(imuTime < firstImageTime)
        {
            imu_msg_used_cnt++;
            continue;
        }
        if (imuTime > timeBond) break;

        // publish info 
        pose_est_new::DataCollectionForSim msgDataCollectionForSim;
        
        msgDataCollectionForSim.header.stamp = imu->header.stamp;
        msgDataCollectionForSim.angular_velocity_imu = imu->angular_velocity;
        msgDataCollectionForSim.linear_acceleration_imu = imu->linear_acceleration;
        
        if(imu_next->header.stamp.toSec() > timeBond)    // 当前数据是相机数据的更新时刻
        {
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
            msgDataCollectionForSim.orientation_cam.w = 0.0;
            msgDataCollectionForSim.orientation_cam.x = 0.0;
            msgDataCollectionForSim.orientation_cam.y = 0.0;
            msgDataCollectionForSim.orientation_cam.z = 0.0;
            msgDataCollectionForSim.pos_cam.x = 0.0;
            msgDataCollectionForSim.pos_cam.y = 0.0;
            msgDataCollectionForSim.pos_cam.z = 0.0;
            msgDataCollectionForSim.updateCameraState = 0;
        }
        dataCollectionForSimPub.publish(msgDataCollectionForSim);
        imu_msg_used_cnt++;
    }
    imuMsgBuffer.erase(imuMsgBuffer.begin(), imuMsgBuffer.begin() + imu_msg_used_cnt);
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
