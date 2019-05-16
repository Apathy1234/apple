#include <pose_est_new/camera_pose.h>

namespace slam_mono
{

CameraPose::CameraPose(void):id(0), q(Quaterniond(1, 0, 0, 0)), p(Vector3d(0, 0, 0)), time(0)
{

}

CameraPose::~CameraPose()
{

}

void CameraPose::Bundle_Adjustment(const vector<Vector3d>& pts1, const vector<Vector3d>& pts2)
{
    id++;
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> > Block;  // pose 6, landmark 3
    typedef g2o::LinearSolverEigen<Block::PoseMatrixType> linerSolver;
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<Block>(g2o::make_unique<linerSolver>()));
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // vertex 当前帧和上一帧的位姿节点
    for( int i = 0; i < 2; i++)
    {
        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId(i);
        if ( i == 0 )
        {
            pose->setFixed(true);
        }
        pose->setEstimate(g2o::SE3Quat());
        optimizer.addVertex(pose);
    }

    // 特征点的节点->路标节点, 以第一帧为准
    for (int i = 0; i < pts1.size(); i++)
    {
        g2o::VertexSBAPointXYZ* landmark = new g2o::VertexSBAPointXYZ();
        landmark->setId(2 + i);
        landmark->setMarginalized(true);
        landmark->setEstimate(pts1[i]);
        optimizer.addVertex(landmark);
    }

    // 准备边 -> 第一帧
    vector<EdgeProjectXYZStoreo*> edges;
    for (int i = 0; i < pts1.size(); i++)
    {
        EdgeProjectXYZStoreo* edge = new EdgeProjectXYZStoreo();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0)));
        edge->setMeasurement(pts1[i]);
        edge->setInformation(Eigen::Matrix3d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    // 第二帧
    for ( int i = 0; i < pts2.size(); i++ )
    {
        EdgeProjectXYZStoreo* edge = new EdgeProjectXYZStoreo();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(1)));
        edge->setMeasurement(pts2[i]);
        edge->setInformation(Eigen::Matrix3d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    // cout << "开始优化..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    // cout << "优化完毕!" << endl;

    // for (int i = 0; i < pts1.size(); i++)
    // {
    //     g2o::VertexSBAPointXYZ* point = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
    //     Vector3d pos = point->estimate();
    //     cout << "raw data: " << pts1[i](0) << ", " << pts1[i](1) << ", " << pts1[i](2) << endl;
    //     cout << "vertex id: " << i + 2 << ", pos: " << pos(0) << ", " << pos(1) << ", " << pos(2) << endl;
    // }



    // optimizer.save("/home/tg/g2o_data/ba.g2o");
    g2o::VertexSE3Expmap* pose = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(1));
    q = pose->estimate().rotation();
    q.normalize();
    p = pose->estimate().translation();
    optimizer.clear();
    static Quaterniond atti = Quaterniond(1, 0, 0, 0);
    static Vector3d pos = Vector3d::Zero();
    atti = q * atti;
    atti.normalize();
    pos = -atti.toRotationMatrix().transpose() * p + pos;
    // cout << "R:" << endl << atti.toRotationMatrix() << endl;
    cout << "pos:" << pos(0) << ", " << pos(1) << ", " << pos(2) << endl;
    // cout << "pose: " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << endl;
    // cout << "pos: " << p(0) << ", " << p(1) << ", " << p(2) << endl;
}
}
