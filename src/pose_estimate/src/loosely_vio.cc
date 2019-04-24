#include <pose_estimate/loosely_vio.h>

namespace slam_mono
{

Loosely_vio::Loosely_vio(void): isSensorCalibr(false), isFirstData(true), trackRate(0), matchLessRatio(0.1)
{

    Load_Parameters();

    featureSub = n.subscribe(FEATURE_TOPICS, 1, &Loosely_vio::Feature_Callback, this);
    imuSub = n.subscribe(IMU_TOPICS, 50, &Loosely_vio::Imu_Callback, this);
    posePub = n.advertise<pose_estimate::PoseEstimateResult>("/camera_pose", 1);
    imuPosePub = n.advertise<geometry_msgs::PoseStamped>("imu_pose", 1);
    ROS_INFO("pose estimate init success! ");  
}

Loosely_vio::~Loosely_vio(void)
{
    
}

void Loosely_vio::Load_Parameters(void)
{
    n.param<string>("imu_topics", IMU_TOPICS, string("/mynteye/imu/data_raw_processed"));
    n.param<string>("pub_feature_topic", FEATURE_TOPICS, string("/feature_tracker/points"));
    n.param<bool>("fixed_bias", params.fixed_bias, false);
    n.param<bool>("fixed_calibr", params.fixed_calibr, false);
    n.param<double>("gyro_noise", params.gyro_noise, 0.005);
    n.param<double>("acc_noise", params.acc_noise, 0.05);
    n.param<double>("gyro_bias_noise", params.gyro_bias_noise, 0.001);
    n.param<double>("acc_bias_noise", params.acc_bias_noise, 0.01);
    n.param<double>("noise_camera_q", params.camear_q_noise, 0.1);
    
    params.gyro_noise *= params.gyro_noise;
    params.acc_noise *= params.acc_noise;
    params.gyro_bias_noise *= params.gyro_bias_noise;
    params.acc_bias_noise *= params.acc_bias_noise;
    params.camear_q_noise *= params.camear_q_noise;

    // init the state_cov
    state_delay.state_cov = Eigen::Matrix<double, STATE_NUM, STATE_NUM>::Zero();
    for(int i = 3; i < 6; i++)                  // bw
        state_delay.state_cov(i, i) = 1e-4;
    for (int i = 6; i < 9; i++)                 // q_wv
        state_delay.state_cov(i, i) = 3e-4;
    for(int i = 9; i < 12; i++)                 // q_ci
        state_delay.state_cov(i, i) = 3e-4;
    // for(int i = 12; i < 15; i++)                // p_ci
    //     state_delay.state_cov(i, i) = 1e-4;

    // init Qc
    params.continues_noise_cov = Eigen::Matrix<double, 6, 6>::Zero();
    params.continues_noise_cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * params.gyro_noise;
    params.continues_noise_cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * params.gyro_bias_noise;

    state_delay.bw = Eigen::Vector3d::Zero();
    state_delay.q_wv = Eigen::Quaterniond(0, 1, 0, 0);
    state_delay.q_ci = state_delay.q_wv.conjugate();
}

void Loosely_vio::Imu_Callback(const sensor_msgs::ImuConstPtr& imuMsg)
{
    imuMsgBuffer.push_back(*imuMsg);
    if(!isSensorCalibr)
    {
        if( imuMsgBuffer.size() < 200 ) return;
        Init_Gravity_Bias();
        isSensorCalibr = true;
    }
    // ROS_INFO_STREAM("q_init: " << state_delay.q.w() << " " << state_delay.q.vec());
}

void Loosely_vio::Init_Gravity_Bias(void)
{
    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();

    for (const auto& imu : imuMsgBuffer)
    {
        Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();

        tf::vectorMsgToEigen(imu.angular_velocity, gyro);
        tf::vectorMsgToEigen(imu.linear_acceleration, acc);

        gyro_sum += gyro;
        acc_sum += acc;
    }
    params.gyro_bias = gyro_sum / imuMsgBuffer.size();

    Eigen::Vector3d gravity = acc_sum / imuMsgBuffer.size();
    double gravity_norm = gravity.norm();
    params.gravity = Eigen::Vector3d(0, gravity_norm, 0);

    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(gravity, params.gravity);

    state_delay.q = q;
}

Eigen::Vector3d Loosely_vio::Calculate_World_Point(const Eigen::Vector3d pts)
{
    // calculat the transform from camera to vision
    Eigen::Isometry3d T_w2c;
    Eigen::Isometry3d T_c2w;            
    // ROS_INFO_STREAM("q: "  << cameraState.orientation);
    T_w2c.linear() = quaternionToRotation(cameraState.orientation);
    T_w2c.translation() = cameraState.position;
    // ROS_INFO_STREAM("rotation_w2c: "  << T_c2w.linear());
    // ROS_INFO_STREAM("translation_w2c: " << T_c2w.translation());
    T_c2w = T_w2c.inverse();
    // ROS_INFO_STREAM("rotation: "  << T_c2w.linear());
    // ROS_INFO_STREAM("translation: " << T_c2w.translation());
    // convert the pts in camera to the vision frame, return the result
    return (T_c2w.linear() * pts + T_c2w.translation());
}

void Loosely_vio::Add_Feature_Points(const feature_tracker::CameraTrackerResultPtr& pts)
{
    int featureNumCurr = mapServer.size();
    int trackedFeatureNum = 0;

    for(const auto& feature : pts->features)
    {
        // ROS_INFO_STREAM("times: " << mapServer[feature.id].keepTimes);
        if (mapServer.find(feature.id) == mapServer.end() && feature.z > 0)
        {
            ROS_INFO_STREAM("camera: " << feature.x << " " << feature.y << " " << feature.z);
            // this is a new feature and the depth is correct
            Eigen::Vector3d worldPts;
            worldPts = Calculate_World_Point(Eigen::Vector3d(feature.x, feature.y, feature.z));
            ROS_INFO_STREAM("world: " << worldPts(0) << " " << worldPts(1) << " " <<worldPts(2));
            mapServer[feature.id] = FeatureState(worldPts);
            
        }
        else
        {
            // this is a old feature
            trackedFeatureNum++;
        }
    }

    trackRate = static_cast<double>(trackedFeatureNum) / 
                static_cast<double>(featureNumCurr);
    // ROS_INFO_STREAM("tracked rate: " << trackRate);
    
}

void Loosely_vio::Find_Feature_Matched(const feature_tracker::CameraTrackerResultPtr& pts)
{
    matchedFeatureID.clear();
    ptsMapMatched.clear();
    ptsCurrMatched.clear();
    pointMatchedMap.clear();
    for(const auto& feature : pts->features)
    {
        //　find the same point in local map
        map<FeatureIDType, FeatureState>::iterator it;
        it = mapServer.find(feature.id);
        if(it != mapServer.end())
        {
            it->second.matchTimes++;
            matchedFeatureID.push_back(feature.id);
            ptsMapMatched.push_back(it->second.position);
            ptsCurrMatched.push_back(Eigen::Vector3d(feature.x, feature.y, feature.z));
            pointMatchedMap[feature.id] = 1;
        }
    }
    // ROS_INFO_STREAM("The number of matching points found is: " << ptsCurrMatched.size());
}

void Loosely_vio::Bundle_Adjustment(void)
{
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> > Block; // pose 6, landmark 3
    typedef g2o::LinearSolverEigen<Block::PoseMatrixType> linerSolver;
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<Block>(g2o::make_unique<linerSolver>()));
    optimizer.setAlgorithm(solver);

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0)));
    optimizer.addVertex(pose);

    // edges
    int index = 1;
    vector<EdgeProjectXYZPose*> edges;
    for (int i = 0; i < ptsCurrMatched.size(); i++)
    {
        EdgeProjectXYZPose* edge = new EdgeProjectXYZPose(ptsMapMatched[i]);
        edge->setId(index++);
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(pose));
        edge->setMeasurement(ptsCurrMatched[i]);
        edge->setInformation(Eigen::Matrix3d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    Eigen::Quaterniond qTemp;
    qTemp = pose->estimate().rotation();
    cameraState.orientation = Eigen::Vector4d(qTemp.x(), qTemp.y(), qTemp.z(), qTemp.w());
    cameraState.position = pose->estimate().translation();
    // ROS_INFO_STREAM("orientation: " << cameraState.orientation);
    // ROS_INFO_STREAM("position: " << cameraState.position);
}

void Loosely_vio::Remove_Feature_Points(void)
{
    vector<FeatureIDType> invalidFeature_id(0);

    for(auto iter = mapServer.begin(); iter != mapServer.end(); iter++ )
    {
        iter->second.keepTimes++;
        auto& feature = iter->second;
        // the point still being tracked.
        if (pointMatchedMap.find(iter->first) != pointMatchedMap.end()) continue;

        float matchRatio = static_cast<float>(iter->second.matchTimes) / 
                           static_cast<float>(iter->second.keepTimes);
        // ROS_INFO_STREAM("id: " << iter->first << " " << "matchRatio: " << matchRatio);
        if(matchRatio < matchLessRatio)
        {
            invalidFeature_id.push_back(iter->first);
            continue;
        }
    }
    for (const auto& feature_id : invalidFeature_id)
    {
        mapServer.erase(feature_id);
    }
    if(mapServer.size() > 1000)
    {
        matchLessRatio += 0.05;
    }
    else
    {
        matchLessRatio = 0.1;
    }
}

void Loosely_vio::Propagate_Model(const double& time, const Eigen::Vector3d& gyro_mes)
{
    Eigen::Vector3d gyro_debias = gyro_mes - params.gyro_bias;
    double dtime = time - params.time;
    ROS_INFO_STREAM("dtime: " << dtime);
    Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
    Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro_debias);
    Omega.block<3, 1>(0, 3) = gyro_debias;
    Omega.block<1, 3>(3, 0) = -gyro_debias;
    ROS_INFO_STREAM("Omega: " << Omega);
    Eigen::Vector4d q = Eigen::Vector4d(state_delay.q.x(), state_delay.q.y(), state_delay.q.z(), state_delay.q.w());

    Eigen::Vector4d dq_dt, dq_dt2;
    double gyro_norm = gyro_debias.norm();
    if(gyro_norm > 1e-5)
    {
        dq_dt = (cos(gyro_norm*dtime*0.5)*Eigen::Matrix4d::Identity() + 1/gyro_norm*sin(gyro_norm*dtime*0.5)*Omega) * q;
        dq_dt2 = (cos(gyro_norm*dtime*0.25)*Eigen::Matrix4d::Identity() + 1/gyro_norm*sin(gyro_norm*dtime*0.25)*Omega) * q;
    }
    else 
    {
        dq_dt = (Eigen::Matrix4d::Identity()+0.5*dtime*Omega) * cos(gyro_norm*dtime*0.5) * q;
        dq_dt2 = (Eigen::Matrix4d::Identity()+0.25*dtime*Omega) * cos(gyro_norm*dtime*0.25) * q;
    }
    ROS_INFO_STREAM("dq_dt: " << dq_dt);
    state_delay.q = Eigen::Quaterniond(dq_dt(3), dq_dt(0), dq_dt(1), dq_dt(2));
    state_delay.q.normalize();
    params.time = time;
}


void Loosely_vio::Process_Model(const double& timeBond)
{
    int imu_msg_used_cnt = 0;

    for(const auto& imu : imuMsgBuffer)
    {
        double imuTime = imu.header.stamp.toSec();
        if( imuTime < params.time)
        {
            imu_msg_used_cnt++;
            continue;
        }
        if( imuTime > timeBond ) break;

        Eigen::Vector3d gyro_mes, acc_mes;
        tf::vectorMsgToEigen(imu.angular_velocity, gyro_mes);
        // tf::vectorMsgToEigen(imu.linear_acceleration, acc_mes);

        /********************************************************/
        Propagate_Model(imuTime, gyro_mes);

        /********************************************************/
        imu_msg_used_cnt++;
    }
    
    imuMsgBuffer.erase(imuMsgBuffer.begin(), imuMsgBuffer.begin() + imu_msg_used_cnt);
}

void Loosely_vio::Feature_Callback(const feature_tracker::CameraTrackerResultPtr& pts)
{
    if(!isSensorCalibr) return;
    
    if ( isFirstData )
    {
        isFirstData = false;
        Add_Feature_Points(pts);
        params.time = pts->header.stamp.toSec();
    }
    else
    {
        Find_Feature_Matched(pts);
        Bundle_Adjustment();
        Remove_Feature_Points();
        // ROS_INFO_STREAM("the map before add: " << mapServer.size());
        Add_Feature_Points(pts);
        // ROS_INFO_STREAM("the map after add: " << mapServer.size());
        

        // fusion start

        Process_Model(pts->header.stamp.toSec());




        /*******************************publish info************************************************/
        pose_estimate::PoseEstimateResultPtr cameraPoseInfo(new pose_estimate::PoseEstimateResult);
        cameraPoseInfo->header.stamp = pts->header.stamp;
        cameraPoseInfo->header.frame_id = "camera";
        
        // cameraPoseInfo->q0 = cameraState.orientation(3);
        // cameraPoseInfo->q1 = cameraState.orientation(0);
        // cameraPoseInfo->q2 = cameraState.orientation(1);
        // cameraPoseInfo->q3 = cameraState.orientation(2);
        cameraPoseInfo->q0 = state_delay.q.w();
        cameraPoseInfo->q1 = state_delay.q.x();
        cameraPoseInfo->q2 = state_delay.q.y();
        cameraPoseInfo->q3 = state_delay.q.z();
        cameraPoseInfo->tx = cameraState.position(0);
        cameraPoseInfo->ty = cameraState.position(1);
        cameraPoseInfo->tz = cameraState.position(2);
        
        posePub.publish(cameraPoseInfo);

        geometry_msgs::PoseStampedPtr imuPoseInfo(new geometry_msgs::PoseStamped);
        imuPoseInfo->header.stamp = pts->header.stamp;
        imuPoseInfo->header.frame_id = "imu";


        imuPoseInfo->pose.orientation.x = state_delay.q.x();
        imuPoseInfo->pose.orientation.y = state_delay.q.y();
        imuPoseInfo->pose.orientation.z = state_delay.q.z();
        imuPoseInfo->pose.orientation.w = state_delay.q.w();

        imuPosePub.publish(imuPoseInfo);
    }
}

/*namespace end*/
}
