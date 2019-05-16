#include <pose_est_new/pose_est_new.h>
#include <pose_est_new/CameraState.h>
#include <pose_est_new/DataCollectionForSim.h>

namespace slam_mono
{

PoseEst::PoseEst(void): isSensorCalibr(false), camera(new CameraPose), isFirstImage(true)
{
    n.param<string>("imu_topics", IMU_TOPIC, string("/mynteye/imu/data_raw_processed"));
    n.param<string>("pub_feature_topic", FEATURE_TOPIC, string("/feature_tracker/points"));
    
    vector<double> t_left2imu_temp;
    // read T_left2imu from yaml
    n.getParam("T_left2imu", t_left2imu_temp);
    if(t_left2imu_temp.size() != 16)    ROS_WARN("invalid vector16 in pose_est_new!");
    cv::Mat T_left2imu_cv = cv::Mat(t_left2imu_temp).clone().reshape(1, 4);
    T_left2imu(0, 0) = T_left2imu_cv.at<double>(0, 0);
    T_left2imu(0, 1) = T_left2imu_cv.at<double>(0, 1);
    T_left2imu(0, 2) = T_left2imu_cv.at<double>(0, 2);
    T_left2imu(0, 3) = T_left2imu_cv.at<double>(0, 3);
    T_left2imu(1, 0) = T_left2imu_cv.at<double>(1, 0);
    T_left2imu(1, 1) = T_left2imu_cv.at<double>(1, 1);
    T_left2imu(1, 2) = T_left2imu_cv.at<double>(1, 2);
    T_left2imu(1, 3) = T_left2imu_cv.at<double>(1, 3);
    T_left2imu(2, 0) = T_left2imu_cv.at<double>(2, 0);
    T_left2imu(2, 1) = T_left2imu_cv.at<double>(2, 1);
    T_left2imu(2, 2) = T_left2imu_cv.at<double>(2, 2);
    T_left2imu(2, 3) = T_left2imu_cv.at<double>(2, 3);
    T_left2imu(3, 0) = T_left2imu_cv.at<double>(3, 0);
    T_left2imu(3, 1) = T_left2imu_cv.at<double>(3, 1);
    T_left2imu(3, 2) = T_left2imu_cv.at<double>(3, 2);
    T_left2imu(3, 3) = T_left2imu_cv.at<double>(3, 3);
    cout << "T_left2imu: " << endl << T_left2imu << endl;

    // read T_sensor form yaml
    vector<double> t_sensor_temp;
    n.getParam("T_sensor", t_sensor_temp);
    if (t_sensor_temp.size()!=9) ROS_WARN("invalid vector9 in pose_est_new!");
    cv::Mat T_sensor_cv = cv::Mat(t_sensor_temp).clone().reshape(1, 3);
    T_sensor(0, 0) = T_sensor_cv.at<double>(0, 0);
    T_sensor(0, 1) = T_sensor_cv.at<double>(0, 1);
    T_sensor(0, 2) = T_sensor_cv.at<double>(0, 2);
    T_sensor(1, 0) = T_sensor_cv.at<double>(1, 0);
    T_sensor(1, 1) = T_sensor_cv.at<double>(1, 1);
    T_sensor(1, 2) = T_sensor_cv.at<double>(1, 2);
    T_sensor(2, 0) = T_sensor_cv.at<double>(2, 0);
    T_sensor(2, 1) = T_sensor_cv.at<double>(2, 1);
    T_sensor(2, 2) = T_sensor_cv.at<double>(2, 2);
    cout << "T_sensor: " << endl << T_sensor << endl;

    // load filter params
    // for P
    n.param<double>("quat_uncertainty", params.quat_uncertainty, 0.1);
    n.param<double>("pose_uncertainty_m", params.pose_uncertainty_m, 10);
    n.param<double>("vel_uncertainty_mps", params.vel_uncertainty_mps, 10);
    n.param<double>("gyro_bias_noise", params.gyro_bias_noise, 0.001);
    n.param<double>("acc_bias_noise", params.acc_bias_noise, 0.01);
    params.quat_uncertainty *= params.quat_uncertainty;
    params.pose_uncertainty_m *= params.pose_uncertainty_m;
    params.vel_uncertainty_mps *= params.vel_uncertainty_mps;
    params.gyro_bias_noise *= params.gyro_bias_noise;
    params.acc_bias_noise *= params.acc_bias_noise;
    // for Q
    n.param<double>("quat_process_noise", params.quat_process_noise, 0.02);
    n.param<double>("pos_process_noise_m", params.pos_process_noise_m, 0.5);
    n.param<double>("vel_process_noise_mps", params.vel_process_noise_mps, 0.2);
    n.param<double>("gyro_process_noise_rps", params.gyro_process_noise_rps, 0.005);
    n.param<double>("acc_process_noise_mps2", params.acc_process_noise_mps2, 0.001);
    params.quat_process_noise *= params.quat_process_noise;
    params.pos_process_noise_m *= params.pos_process_noise_m;
    params.vel_process_noise_mps *= params.vel_process_noise_mps;
    params.gyro_process_noise_rps *= params.gyro_process_noise_rps;
    params.acc_process_noise_mps2 *= params.acc_process_noise_mps2;
    //for R
    n.param<double>("cam_trans_means_m", params.cam_trans_means_m, 0.05);
    n.param<double>("cam_pose_means", params.cam_pose_means, 0.5);
    n.param<double>("imu_acc_means_mps2", params.imu_acc_means_mps2, 0.1);
    params.cam_trans_means_m *= params.cam_trans_means_m;
    params.cam_pose_means *= params.cam_pose_means;
    params.imu_acc_means_mps2 *= params.imu_acc_means_mps2;
    ROS_INFO("parameters load success!");

    state_delay.q = Quaterniond(1, 0 ,0, 0);
    state_delay.p = Vector3d(0, 0, 0);
    state_delay.v = Vector3d(0, 0, 0);
    state_delay.bw = Vector3d(0, 0, 0);
    state_delay.ba = Vector3d(0, 0, 0);

    // init P
    state_delay.state_cov = Matrix<double, STATE_NUM, STATE_NUM>::Zero();
    for (int i = 0; i < 4; i++)             // quat
        state_delay.state_cov(i, i) = params.quat_uncertainty;
    for (int i = 4; i < 7; i++)             // pos
        state_delay.state_cov(i, i) = params.pose_uncertainty_m;
    for (int i = 7; i < 10; i++)            // vel
        state_delay.state_cov(i, i) = params.vel_uncertainty_mps;
    for (int i = 10; i < 13; i++)           // bw
        state_delay.state_cov(i, i) = params.gyro_bias_noise;
    for (int i = 13; i < 16; i++)           // ba
        state_delay.state_cov(i, i) = params.acc_bias_noise;
    // init Q
    state_delay.Q = Matrix<double, STATE_NUM, STATE_NUM>::Zero();
    for (int i = 0; i < 4; i++)             // quat
        state_delay.Q(i, i) = params.quat_process_noise;
    for (int i = 4; i < 7; i++)             // pos
        state_delay.Q(i, i) = params.pos_process_noise_m;
    for (int i = 7; i < 10; i++)            // vel
        state_delay.Q(i, i) = params.vel_process_noise_mps;
    for (int i = 10; i < 13; i++)           // bw
        state_delay.Q(i, i) = params.gyro_process_noise_rps;
    for (int i = 13; i < 16; i++)           // ba
        state_delay.Q(i, i) = params.acc_process_noise_mps2;
    // init R
    state_delay.R_accel = Matrix<double, 3, 3>::Zero();
    for (int i = 0; i < 3; i++)
        state_delay.R_accel(i, i) = params.imu_acc_means_mps2;
    
    state_delay.R_cam = Matrix<double, MEASURE_NUM, MEASURE_NUM>::Zero();
    for (int i = 0; i < 4; i++)
        state_delay.R_cam(i, i) = params.cam_pose_means;
    for (int i = 4; i < 7; i++)
        state_delay.R_cam(i, i) = params.cam_trans_means_m;

    // cout << "P: " << endl << state_delay.state_cov << endl;
    // cout << "Q: " << endl << state_delay.Q << endl;
    // cout << "R_accel: " << endl << state_delay.R_accel << endl;
    // cout << "R_cam: " << endl << state_delay.R_cam << endl;
    ROS_INFO("init filter success!");

    featureSub = n.subscribe(FEATURE_TOPIC, 1, &PoseEst::Feature_Callback, this);
    imuSub = n.subscribe(IMU_TOPIC, 50, &PoseEst::Imu_Callback, this);

    cameraStatePub = n.advertise<pose_est_new::CameraState>("/slam_mono/cameraState", 1);
    dataCollectionForSimPub = n.advertise<pose_est_new::DataCollectionForSim>("/slam_mono/dataCollectionForSim",50);
    ROS_INFO("create ros IO success!");
}

PoseEst::~PoseEst()
{

}

void PoseEst::Imu_Callback(const sensor_msgs::ImuConstPtr& msg)
{
    imuMsgBuffer.push_back(*msg);
    if(!isSensorCalibr) 
    {
        if(imuMsgBuffer.size() < 200) return;
        Init_Sensor();
        isSensorCalibr = true;
    }
}

void PoseEst::Feature_Callback(const feature_tracker::CameraTrackerResultConstPtr& msg)
{
    double timeBegin = ros::Time::now().toNSec();
    // 传感器校准没有完成，等待传感器校准
    if (!isSensorCalibr) return;
    // 清除当前特征点信息
    Clear_Points(featuresCurr);
    // 将当前特征点放入容器
    Put_Feature_into_Vector(msg);

    if (isFirstImage)
    {
        isFirstImage = false;
        params.time = msg->header.stamp.toSec();
    }
    else
    {
        Find_Feature_Matches();
        camera->Bundle_Adjustment(ptsRefMatched, ptsCurrMatched);
        Deal_with_IMU(msg->header.stamp.toSec());
    }
    
    featuresRef = featuresCurr;

    double timeEnd = ros::Time::now().toNSec();
    // ROS_INFO_STREAM("time cost: " << (timeEnd - timeBegin) << " ns");
}

void PoseEst::Init_Sensor(void)
{
    Vector3d gyro_sum = Vector3d::Zero();
    Vector3d accel_sum = Vector3d::Zero();

    for (const auto imu : imuMsgBuffer)
    {
        Vector3d gyro_temp = Vector3d::Zero();
        Vector3d accel_temp = Vector3d::Zero();

        tf::vectorMsgToEigen(imu.angular_velocity, gyro_temp);
        tf::vectorMsgToEigen(imu.linear_acceleration, accel_temp);

        gyro_sum += gyro_temp;
        accel_sum += accel_temp;
    }
    // save the bias of gyro
    params.gyro_bias = gyro_sum / imuMsgBuffer.size();
    
    // init the orientation
    Vector3d accel_temp = accel_sum / imuMsgBuffer.size();
    double gravity = accel_temp.norm();
    params.gravity = Vector3d(0, 0, -gravity);
    Vector3d accel_used = T_sensor * accel_temp;
    // cout << params.gravity << endl << accel_used << endl;
    Quaterniond q_b2ned = Quaterniond::FromTwoVectors(accel_used, params.gravity);
    state_delay.q = q_b2ned.inverse();
    cout << "state_delay.q_init: " << state_delay.q.w() << ", " << state_delay.q.x() << ", " << state_delay.q.y() << ", " << state_delay.q.z() << endl;


}

void PoseEst::Process_Model(const double& time, Vector3d& gyro_mes, Vector3d& acc_mes)
{
    Vector3d gyro_debias = gyro_mes - params.gyro_bias - state_delay.bw;
    Vector3d acc_debias = acc_mes - state_delay.ba;
    double dtime = time - params.time;
    // current atti
    double q0 = state_delay.q.w(); 
    double q1 = state_delay.q.x();
    double q2 = state_delay.q.y();
    double q3 = state_delay.q.z();
    
    // calculate Omega
    Matrix4d Omega = Matrix4d::Zero();
    Omega.block<1, 3>(0, 1) = -gyro_debias;
    Omega.block<3, 1>(1, 0) = gyro_debias;
    Omega.block<3, 3>(1, 1) = -skewSymmetric(gyro_debias);

    
    // propagate state
    Vector4d dq_dt;
    Vector4d q = Vector4d(state_delay.q.w(), state_delay.q.x(), state_delay.q.y(), state_delay.q.z());
    double gyro_norm = gyro_debias.norm();
    if(gyro_norm > 1e-5)
    {
        dq_dt = (cos(gyro_norm*dtime*0.5)*Matrix4d::Identity() + 1/gyro_norm*sin(gyro_norm*dtime*0.5)*Omega) * q;
    }
    else
    {
        dq_dt = (Matrix4d::Identity()+0.5*dtime*Omega) * cos(gyro_norm*dtime*0.5) * q;
    }
    
    state_delay.q = Quaterniond(dq_dt(0), dq_dt(1), dq_dt(2), dq_dt(3));;
    state_delay.q.normalize();
    // cout << "state_delay.q: " << state_delay.q.w() << ", " << state_delay.q.x() << ", " << state_delay.q.y() << ", " << state_delay.q.z() << endl;
    
    // calculate F
    Matrix<double, STATE_NUM, STATE_NUM> F = Matrix<double, STATE_NUM, STATE_NUM>::Zero();


    params.time = time;
}

void PoseEst::Deal_with_IMU(const double& timeBond)
{
    int imu_msg_used_cnt = 0;
    
    for( vector<sensor_msgs::Imu>::iterator imu = imuMsgBuffer.begin(); imu != imuMsgBuffer.end(); imu++)
    {
        vector<sensor_msgs::Imu>::iterator imu_next = imu + 1;
        
        double imuTime = imu->header.stamp.toSec();
        if(imuTime < params.time)
        {
            imu_msg_used_cnt++;
            continue;
        }
        if (imuTime > timeBond) break;

        Vector3d gyro_mes, acc_mes;
        tf::vectorMsgToEigen(imu->angular_velocity, gyro_mes);
        tf::vectorMsgToEigen(imu->linear_acceleration, acc_mes);

        Process_Model(imuTime, gyro_mes, acc_mes);
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
    // ROS_INFO_STREAM("the number of matching points found is: " << ptsRefMatched.size());
}

}
