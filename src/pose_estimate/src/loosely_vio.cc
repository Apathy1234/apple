#include <pose_estimate/loosely_vio.h>

namespace slam_mono
{

Loosely_vio::Loosely_vio(void): isSensorCalibr(false), isFirstData(true), trackRate(0), matchLessRatio(0.1)
{
    n.param<string>("imu_topics", IMU_TOPICS, string("/mynteye/imu/data_raw_processed"));
    n.param<string>("pub_feature_topic", FEATURE_TOPICS, string("/feature_tracker/points"));
    
    featureSub = n.subscribe(FEATURE_TOPICS, 1, &Loosely_vio::Feature_Callback, this);
    // imuSub = n.subscribe(IMU_TOPICS, 50, &Loosely_vio::Imu_Callback, this);
    posePub = n.advertise<pose_estimate::PoseEstimateResult>("/camera_pose", 1);
    ROS_INFO("pose estimate init success! ");  
}

Loosely_vio::~Loosely_vio(void)
{

}

Eigen::Vector3d Loosely_vio::Calculate_World_Point(const Eigen::Vector3d pts)
{
    // calculat the transform from camera to vision
    Eigen::Isometry3d T_w2c;
    Eigen::Isometry3d T_c2w;            
    ROS_INFO_STREAM("q: "  << cameraState.orientation);
    T_w2c.linear() = quaternionToRotation(cameraState.orientation);
    T_w2c.translation() = cameraState.position;
    ROS_INFO_STREAM("rotation_w2c: "  << T_c2w.linear());
    ROS_INFO_STREAM("translation_w2c: " << T_c2w.translation());
    T_c2w = T_w2c.inverse();
    ROS_INFO_STREAM("rotation: "  << T_c2w.linear());
    ROS_INFO_STREAM("translation: " << T_c2w.translation());
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
    ROS_INFO_STREAM("tracked rate: " << trackRate);
    
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
    ROS_INFO_STREAM("The number of matching points found is: " << ptsCurrMatched.size());
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

void Loosely_vio::Feature_Callback(const feature_tracker::CameraTrackerResultPtr& pts)
{
    // if(!isSensorCalibr) return;
    
    if ( isFirstData )
    {
        isFirstData = false;
        Add_Feature_Points(pts);
    }
    else
    {
        Find_Feature_Matched(pts);
        Bundle_Adjustment();
        Remove_Feature_Points();
        // ROS_INFO_STREAM("the map before add: " << mapServer.size());
        Add_Feature_Points(pts);
        // ROS_INFO_STREAM("the map after add: " << mapServer.size());
        
        /*******************************publish info************************************************/
        pose_estimate::PoseEstimateResultPtr cameraPoseInfo(new pose_estimate::PoseEstimateResult);
        cameraPoseInfo->header.stamp = pts->header.stamp;
        cameraPoseInfo->header.frame_id = "camera";
        
        cameraPoseInfo->q0 = cameraState.orientation(3);
        cameraPoseInfo->q1 = cameraState.orientation(0);
        cameraPoseInfo->q2 = cameraState.orientation(1);
        cameraPoseInfo->q3 = cameraState.orientation(2);
        cameraPoseInfo->tx = cameraState.position(0);
        cameraPoseInfo->ty = cameraState.position(1);
        cameraPoseInfo->tz = cameraState.position(2);
        
        posePub.publish(cameraPoseInfo);
    }
}

/*namespace end*/
}
