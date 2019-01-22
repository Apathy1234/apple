#include <pose_estimate/pose_estimate.h>

namespace slam_mono
{

PoseEstimate::PoseEstimate(void)
{
    n.param<string>("imu_topics", IMU_TOPICS, string("/imu"));
    n.param<string>("pub_feature_topic", FEATURE_TOPICS, string("/feature_tracker/points"));
    featureSub = n.subscribe(FEATURE_TOPICS, 1, &PoseEstimate::Feature_Callback, this);
    // imuSub.subscribe(n, IMU_TOPICS, 1);
    ROS_INFO("pose estimate init success! ");
}

PoseEstimate::~PoseEstimate(void)
{
    
}

void PoseEstimate::Clear_Points(Features& fet)
{
    fet.id.clear();
    fet.cnt.clear();
    fet.pts3d.clear();
    fet.pts3dMap.clear();
}

void PoseEstimate::Find_Feature_Matches(void)
{
    ptsRefMatched.clear();
    ptsCurrMatched.clear();
    for(int i = 0; i < featuresCurr.id.size(); i++)
    {
        if(featuresCurr.cnt[i] != 1)   // 并非只追踪了一次
        {
            map<LONGTYPE, Point3f>::iterator it;
            it = featuresRef.pts3dMap.find(featuresCurr.id[i]);
            if( it != featuresRef.pts3dMap.end() )
            {
                ptsRefMatched.push_back(it->second);
                ptsCurrMatched.push_back(featuresCurr.pts3d[i]);
            }
        }
    }
    ROS_INFO_STREAM("The number of matching points found is: " << ptsRefMatched.size());
}

void PoseEstimate::Bundle_Adjustment(const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t)
{
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> > Block;  // pose 6, landmark 3
    typedef g2o::LinearSolverEigen<Block::PoseMatrixType> linerSolver;
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<Block>(g2o::make_unique<linerSolver>()));
    optimizer.setAlgorithm(solver);
    
    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0)));
    optimizer.addVertex( pose );

    // edges
    int index = 1;
    vector<EdgeProjectXYZPose*> edges;
    for ( int i = 0; i < ptsRefMatched.size(); i++ )
    {
        EdgeProjectXYZPose* edge = new EdgeProjectXYZPose(Eigen::Vector3d(ptsCurrMatched[i].x, ptsCurrMatched[i].y, ptsCurrMatched[i].z));
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*>(pose) );
        edge->setMeasurement(Eigen::Vector3d(ptsRefMatched[i].x, ptsRefMatched[i].y, ptsRefMatched[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity()*1e4);
        optimizer.addEdge( edge );
        index++;
        edges.push_back( edge );
    }
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    cout << (Eigen::Isometry3d( pose->estimate() ).matrix()) << endl;
}

void PoseEstimate::Feature_Callback(const feature_tracker::CameraTrackerResultPtr& pts)
{
    uint64 timeBegin = ros::Time::now().toNSec();
    Clear_Points(featuresCurr);

    featuresCurr.header = pts->header;
    for(int i = 0; i < pts->num_of_features; i++)
    {
        featuresCurr.id.push_back(pts->features[i].id);
        featuresCurr.cnt.push_back(pts->features[i].cnt);
        featuresCurr.pts3d.push_back(Point3f(pts->features[i].x, pts->features[i].y, pts->features[i].z));
        featuresCurr.pts3dMap.insert(make_pair(pts->features[i].id, Point3f(pts->features[i].x, pts->features[i].y, pts->features[i].z)));
    }
    if( !featuresRef.pts3dMap.empty() )
    {
        Find_Feature_Matches();
        Bundle_Adjustment(ptsRefMatched, ptsCurrMatched, R, T);
    }
    featuresRef = featuresCurr;
    uint64 timeEnd = ros::Time::now().toNSec();

    cout<<(timeEnd - timeBegin)<< endl;
}


}
