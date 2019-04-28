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

    // init state_delay
    state_delay.bw = Eigen::Vector3d::Zero();
    state_delay.q_wv = Eigen::Quaterniond(0, 1, 0, 0);
    state_delay.q_ci = state_delay.q_wv.conjugate();
    state_delay.xCorrect = Eigen::Matrix<double, STATE_NUM, 1>::Zero();
    state_delay.R = Eigen::Matrix<double, 4, 4>::Identity() * 1 * 1;
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
    state_delay.q.normalize();
    // state_delay.q_wv = (state_delay.q_ci * state_delay.q).conjugate();
    cameraState.orientation = state_delay.q_ci * state_delay.q * state_delay.q_wv;
}

Eigen::Vector3d Loosely_vio::Calculate_World_Point(const Eigen::Vector3d pts)
{
    // calculat the transform from camera to vision
    Eigen::Isometry3d T_w2c;
    Eigen::Isometry3d T_c2w;            
    T_w2c.linear() = cameraState.orientation.toRotationMatrix();//.transpose();  // from vision to camera
    T_w2c.translation() = cameraState.position;
    T_c2w = T_w2c.inverse();                                         // from camera to vision
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
            // ROS_INFO_STREAM("camera: " << feature.x << " " << feature.y << " " << feature.z);
            // this is a new feature and the depth is correct
            Eigen::Vector3d worldPts;
            worldPts = Calculate_World_Point(Eigen::Vector3d(feature.x, feature.y, feature.z));
            // ROS_INFO_STREAM("world: " << worldPts(0) << " " << worldPts(1) << " " <<worldPts(2));
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
    pose->setEstimate( g2o::SE3Quat(state_delay.q_ci * state_delay.q * state_delay.q_wv, Eigen::Vector3d(0, 0, 0)));
    optimizer.addVertex(pose);

    // edges
    int index = 1;
    vector<EdgeProjectXYZPose*> edges;
    for (int i = 0; i < ptsCurrMatched.size(); i++)
    {
        EdgeProjectXYZPose* edge = new EdgeProjectXYZPose(ptsMapMatched[i] / 1000.0f);
        edge->setId(index++);
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(pose));
        edge->setMeasurement(ptsCurrMatched[i] / 1000.0f);
        edge->setInformation(Eigen::Matrix3d::Identity() * 0.25);
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    Eigen::Quaterniond qTemp;
    qTemp = pose->estimate().rotation();
    cameraState.orientation = qTemp;
    cameraState.orientation.normalize();
    cameraState.position = pose->estimate().translation() * 1000.0f;
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

void Loosely_vio::Process_Model(const double& time, const Eigen::Vector3d& gyro_mes)
{
    Eigen::Vector3d gyro_debias = gyro_mes - params.gyro_bias - state_delay.bw;
    double dtime = time - params.time;
    Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
    Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro_debias);
    Omega.block<3, 1>(0, 3) = gyro_debias;
    Omega.block<1, 3>(3, 0) = -gyro_debias;

    // propagate state
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
    state_delay.q = Eigen::Quaterniond(dq_dt(3), dq_dt(0), dq_dt(1), dq_dt(2));
    state_delay.q.normalize();

    // calculate process covariance
    Eigen::Matrix<double, STATE_NUM, STATE_NUM> F = Eigen::Matrix<double, STATE_NUM, STATE_NUM>::Zero();
    Eigen::Matrix<double, STATE_NUM, 6> G = Eigen::Matrix<double, STATE_NUM, 6>::Zero();

    F.block<3, 3>(0, 0) = -skewSymmetric(gyro_debias);
    F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    G.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    G.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

    // calculate Fd
    Eigen::Matrix<double, STATE_NUM, STATE_NUM> Fdt = F * dtime;
    Eigen::Matrix<double, STATE_NUM, STATE_NUM> Fdt_square = Fdt * Fdt;
    Eigen::Matrix<double, STATE_NUM, STATE_NUM> Fdt_cube = Fdt_square * Fdt;

    Eigen::Matrix<double, STATE_NUM, STATE_NUM> Fd = Eigen::Matrix<double, STATE_NUM, STATE_NUM>::Identity() + 
                                                     Fdt + 0.5 * Fdt_square + (1.0 / 6.0) * Fdt_cube;
    
    Eigen::Matrix<double, STATE_NUM, STATE_NUM> Q = Fd * G * params.continues_noise_cov * 
                                                    G.transpose() * F.transpose() * dtime;
    
    state_delay.state_cov = Fd * state_delay.state_cov * Fd.transpose() + Q;

    Eigen::MatrixXd state_cov_symmetry = (state_delay.state_cov + state_delay.state_cov.transpose()) / 2.0;

    state_delay.state_cov = state_cov_symmetry;

    params.time = time;
}


void Loosely_vio::Deal_with_Imu(const double& timeBond)
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

        Process_Model(imuTime, gyro_mes);

        imu_msg_used_cnt++;
    }
    
    imuMsgBuffer.erase(imuMsgBuffer.begin(), imuMsgBuffer.begin() + imu_msg_used_cnt);
}

void Loosely_vio::Measurement_Update(void)
{
    Eigen::Matrix<double, 4, STATE_NUM> H = Eigen::Matrix<double, 4, 12>::Zero();
    Eigen::Matrix<double, 4, 1> zhat = Eigen::Matrix<double, 4, 1>::Zero();

    Eigen::Matrix3d R_wv = state_delay.q_wv.toRotationMatrix();//.transpose();  // from vision to world
    Eigen::Matrix3d R_ci = state_delay.q_ci.toRotationMatrix();//.transpose();  // from imu to camera
    Eigen::Matrix3d R = state_delay.q.toRotationMatrix();//.transpose();        // from world to imu
    
    H.block<3, 3>(0, 0) = R_wv.transpose();
    H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, 9) = R_wv.transpose() * R.transpose();
    H(3, 8) = 1;

    // calculate resudial
    Eigen::Quaterniond Hx = state_delay.q_ci * state_delay.q * state_delay.q_wv;
    Hx.normalize();

    Eigen::Quaterniond q_err_use = Hx.inverse() * cameraState.orientation;

    zhat.block<3, 1>(0, 0) = q_err_use.vec() / q_err_use.w() * 2;
    q_err_use = state_delay.q_wv;
    zhat(3, 0) = -2 * (q_err_use.w() * q_err_use.z() + q_err_use.x() * q_err_use.y()) / 
                 (1 - 2 * (q_err_use.y() * q_err_use.y() + q_err_use.z() * q_err_use.z()));

    Eigen::Matrix<double, 4, 4> S = H * state_delay.state_cov * H.transpose() + state_delay.R;
    Eigen::Matrix<double, STATE_NUM, 4> K = state_delay.state_cov * H.transpose() * S.inverse();
    Eigen::Matrix<double, STATE_NUM, STATE_NUM> I_KH = Eigen::Matrix<double, STATE_NUM, STATE_NUM>::Identity() - K * H;

    // calculate the correct and update the state_cov
    state_delay.xCorrect = K * zhat;
    state_delay.state_cov = I_KH * state_delay.state_cov * I_KH.transpose() + K * state_delay.R * K.transpose();
    
    Eigen::MatrixXd state_cov_symmetry = (state_delay.state_cov + state_delay.state_cov.transpose()) / 2.0;
    state_delay.state_cov = state_cov_symmetry;
}

/*  output normalized q  */
void Loosely_vio::Apply_Correct(void)
{
    Eigen::Matrix<double, STATE_NUM, 1> cor = state_delay.xCorrect;
    if(params.fixed_bias)
    {
        cor.block<3, 1>(3, 0) = Eigen::Vector3d::Zero();
    }
    if(params.fixed_calibr)
    {
        cor.block<6, 1>(6, 0) = Eigen::Matrix<double, 6, 1>::Zero();
    }

    state_delay.bw += cor.block<3, 1>(3, 0);

    Eigen::Quaterniond qbuffer_q = QuaternionFromSmallAngle(cor.block<3, 1>(0, 0));
    state_delay.q = state_delay.q * qbuffer_q;
    state_delay.q.normalize();

    Eigen::Quaterniond qbuffer_q_wv = QuaternionFromSmallAngle(cor.block<3, 1>(6, 0));
    state_delay.q_wv = state_delay.q_wv * qbuffer_q_wv;
    state_delay.q_wv.normalize();

    Eigen::Quaterniond qbuffer_q_ci = QuaternionFromSmallAngle(cor.block<3, 1>(9, 0));
    state_delay.q_ci = state_delay.q_ci * qbuffer_q_ci;
    state_delay.q_ci.normalize();

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
        
        Deal_with_Imu(pts->header.stamp.toSec());
        
        Bundle_Adjustment();

        // fusion start
        Measurement_Update();

        Apply_Correct();

        // deal with map
        Remove_Feature_Points();
        // ROS_INFO_STREAM("the map before add: " << mapServer.size());
        Add_Feature_Points(pts);
        // ROS_INFO_STREAM("the map after add: " << mapServer.size());
        


        /*******************************publish info************************************************/
        double q0 = cameraState.orientation.w();
        double q1 = cameraState.orientation.x();
        double q2 = cameraState.orientation.y();
        double q3 = cameraState.orientation.z();
        Eigen::Matrix3d C_ned2b;
        C_ned2b << 1-2*(q2*q2+q3*q3),      2*(q1*q2+q3*q0),       2*(q1*q3-q2*q0),
                     2*(q1*q2-q3*q0),    1-2*(q1*q1+q3*q3),       2*(q2*q3+q1*q0),
                     2*(q1*q3+q2*q0),      2*(q2*q3-q1*q0),     1-2*(q1*q1+q2*q2);

        double yaw = asin(-C_ned2b(0, 2)) * 57.3;
        double pitch = atan2(C_ned2b(1, 2), C_ned2b(2, 2)) * 57.3;
        double roll = atan2(C_ned2b(0, 1), C_ned2b(0, 0)) * 57.3;
        if (abs(C_ned2b(0,2)) > 1 - 1e-8)
        {
            pitch = 0;
            yaw = atan2( -C_ned2b(0,2), C_ned2b(2,2)) * 57.3;
            roll = atan2( -C_ned2b(1,0), C_ned2b(1,1)) * 57.3;
        }

        q0 = state_delay.q.w();
        q1 = state_delay.q.x();
        q2 = state_delay.q.y();
        q3 = state_delay.q.z();
        C_ned2b << 1-2*(q2*q2+q3*q3),      2*(q1*q2+q3*q0),       2*(q1*q3-q2*q0),
                     2*(q1*q2-q3*q0),    1-2*(q1*q1+q3*q3),       2*(q2*q3+q1*q0),
                     2*(q1*q3+q2*q0),      2*(q2*q3-q1*q0),     1-2*(q1*q1+q2*q2);

        double yaw_state = asin(-C_ned2b(0, 2)) * 57.3;
        double pitch_state = atan2(C_ned2b(1, 2), C_ned2b(2, 2)) * 57.3;
        double roll_state = atan2(C_ned2b(0, 1), C_ned2b(0, 0)) * 57.3;
        if (abs(C_ned2b(0,2)) > 1 - 1e-8)
        {
            pitch_state = 0;
            yaw_state = atan2( -C_ned2b(0,2), C_ned2b(2,2)) * 57.3;
            roll_state = atan2( -C_ned2b(1,0), C_ned2b(1,1)) * 57.3;
        }
        
        pose_estimate::PoseEstimateResultPtr cameraPoseInfo(new pose_estimate::PoseEstimateResult);
        cameraPoseInfo->header.stamp = pts->header.stamp;
        cameraPoseInfo->header.frame_id = "camera";
        
        cameraPoseInfo->q0 = roll_state;
        cameraPoseInfo->q1 = pitch_state;
        cameraPoseInfo->q2 = yaw_state;
        cameraPoseInfo->q3 = cameraState.orientation.z();
        cameraPoseInfo->tx = state_delay.bw(0);
        cameraPoseInfo->ty = state_delay.bw(1);
        cameraPoseInfo->tz = state_delay.bw(2);
        cameraPoseInfo->yaw = yaw;
        cameraPoseInfo->pitch = pitch;
        cameraPoseInfo->roll = roll;
        
        
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
