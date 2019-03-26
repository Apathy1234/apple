#include <feature_tracker/feature_tracker.h>

namespace slam_mono
{
FeatureTracker::FeatureTracker(ros::NodeHandle& nh):n(nh), state(FIRST_IMAGE), pubCnt(1), pubThisFrame(false), firstImageTime(0), currImageTime(0), stereoSub(10)
{
    if( !Load_Parameters() ) return;
    ROS_INFO(" feature_tracker load parameters success! ");

    if ( !Create_RosIO() ) return;
    ROS_INFO(" feature_tracker create ROS IO finished! ");
}

FeatureTracker::~FeatureTracker(void)
{
    
}

bool FeatureTracker:: Load_Parameters(void)
{
    n.param<string>("left/distortion_model", distortionModel, string("radtan"));

    vector<int> cam_resolution_temp;
    n.getParam("left/resolution", cam_resolution_temp);
    camResolution[0] = cam_resolution_temp[0];
    camResolution[1] = cam_resolution_temp[1];

    vector<double> left_intrinsics_temp(4);
    n.getParam("left/intrinsics", left_intrinsics_temp);
    leftIntrinsics[0] = left_intrinsics_temp[0];
    leftIntrinsics[1] = left_intrinsics_temp[1];
    leftIntrinsics[2] = left_intrinsics_temp[2];
    leftIntrinsics[3] = left_intrinsics_temp[3];

    vector<double> right_intrinsics_temp(4);
    n.getParam("right/intrinsics", right_intrinsics_temp);
    rightIntrinsics[0] = right_intrinsics_temp[0];
    rightIntrinsics[1] = right_intrinsics_temp[1];
    rightIntrinsics[2] = right_intrinsics_temp[2];
    rightIntrinsics[3] = right_intrinsics_temp[3];

    vector<double> left_distortion_temp(4);
    n.getParam("left/distortion_coeffs", left_distortion_temp);
    leftDistortionCoeffs[0] = left_distortion_temp[0];
    leftDistortionCoeffs[1] = left_distortion_temp[1];
    leftDistortionCoeffs[2] = left_distortion_temp[2];
    leftDistortionCoeffs[3] = left_distortion_temp[3];

    vector<double> right_distortion_temp(4);
    n.getParam("right/distortion_coeffs", right_distortion_temp);
    rightDistortionCoeffs[0] = right_distortion_temp[0];
    rightDistortionCoeffs[1] = right_distortion_temp[1];
    rightDistortionCoeffs[2] = right_distortion_temp[2];
    rightDistortionCoeffs[3] = right_distortion_temp[3];

    vector<double> t_left2right_temp;
    n.getParam("T_left2right", t_left2right_temp);
    if (t_left2right_temp.size() != 16)
    {
        ROS_WARN(" invalid vector16! ");
    }
    T_left2right = Mat(t_left2right_temp).clone().reshape(1, 4);
    
    r_left2right = T_left2right(Rect(0, 0, 3, 3));
    t_left2right = T_left2right(Rect(3, 0, 1, 3));

    vector<double> t_left2imu_temp;
    n.getParam("T_left2imu", t_left2imu_temp);
    if (t_left2imu_temp.size() != 16)
    {
        ROS_WARN(" invalid vector16! ");
    }
    T_left2imu = Mat(t_left2imu_temp).clone().reshape(1, 4);
    
    r_left2imu = T_left2imu(Rect(0, 0, 3, 3));
    t_left2imu = T_left2imu(Rect(3, 0, 1, 3));

    Mat T_right2left;
    invert(T_left2right, T_right2left);
    Mat T_right2imu = T_left2imu * T_right2left;

    r_right2imu = T_right2imu(Rect(0, 0, 3, 3));
    t_right2imu = T_right2imu(Rect(3, 0, 1, 3));

    n.param<int>("frequence", FREQ, 10);
    n.param<int>("board_size", BOARD_SIZE, 1);
    n.param<int>("tracker_size", TRACKER_SIZE, 30);
    n.param<int>("tracker_min_distance", TRACKER_DIS, 20);
    n.param<int>("tracker_max_count", TRACKER_NUM, 100);
    n.param<int>("pyramid_level", PYRAMID_LEVEL, 3);
    n.param<int>("win_size", WIN_SIZE, 15);
    n.param<double>("ransac_threshold", RANSAC_THRESHOLD, 3.0);
    n.param<double>("stereo_threshold", STEREO_THRESHOLD, 5.0);
    n.getParam("left/topic", LEFT_TOPICS);
    n.getParam("right/topic", RIGHT_TOPICS);
    n.getParam("imu_topics", IMU_TOPICS);
    n.getParam("pub_match_image_topic", FEATURE_IMAGE_TOPICS);
    n.getParam("pub_feature_topic", FEATURE_TOPICS);
    n.getParam("equalize", EQUALIZE);
    n.getParam("show_tracker", SHOW_TRACKER);
    

    ROS_INFO("====================================================");
    ROS_INFO("camera_resolution: %d, %d",
        camResolution[0], camResolution[1]);
    ROS_INFO("left_intrinsics: %f, %f, %f, %f",
        leftIntrinsics[0], leftIntrinsics[1], leftIntrinsics[2], leftIntrinsics[3]);
    cout << "left_distortion_model: " << distortionModel << endl;
    ROS_INFO("left_distortion_coeffs: %f, %f, %f, %f",
        leftDistortionCoeffs[0], leftDistortionCoeffs[1], leftDistortionCoeffs[2], leftDistortionCoeffs[3]);
    
    ROS_INFO("right_intrinsics: %f, %f, %f, %f",
        rightIntrinsics[0], rightIntrinsics[1], rightIntrinsics[2], rightIntrinsics[3]);
    cout << "right_distortion_model: " << distortionModel << endl;
    ROS_INFO("right_distortion_coeffs: %f, %f, %f, %f",
        rightDistortionCoeffs[0], rightDistortionCoeffs[1], rightDistortionCoeffs[2], rightDistortionCoeffs[3]);
    
    cout << "r and t from left to right: " <<endl;
    cout << r_left2right << endl;
    cout << t_left2right << endl; 
    
    ROS_INFO("frequence: %d", FREQ);
    ROS_INFO("board_size: %d", BOARD_SIZE);
    ROS_INFO("tracker_size: %d", TRACKER_SIZE);
    ROS_INFO("tracker_min_distance: %d", TRACKER_DIS);
    ROS_INFO("tracker_number: %d", TRACKER_NUM);
    ROS_INFO("pyramid_level: %d", PYRAMID_LEVEL);
    ROS_INFO("win_size: %d", WIN_SIZE);
    ROS_INFO("ransac_threshold: %f", RANSAC_THRESHOLD);
    cout << "left_topics: " << LEFT_TOPICS << endl;
    cout << "right_topics: " << RIGHT_TOPICS << endl;
    cout << "imu_topics: " << IMU_TOPICS << endl;
    cout << "pub_match_image_topic: " << FEATURE_IMAGE_TOPICS << endl;
    cout << "pub_feature_topic: " << FEATURE_TOPICS << endl;
    ROS_INFO("====================================================");
    return true;
}

bool FeatureTracker::Create_RosIO(void)
{
    image_transport::ImageTransport it(n);

    imuSub = n.subscribe(IMU_TOPICS, 50, &FeatureTracker::Imu_Callback, this);

    pubMatchImage = it.advertise(FEATURE_IMAGE_TOPICS, 10);
    pubFeatures = n.advertise<feature_tracker::CameraTrackerResult>(FEATURE_TOPICS, 10);
    leftSub.subscribe(n, LEFT_TOPICS, 1);
    rightSub.subscribe(n, RIGHT_TOPICS, 1);
    stereoSub.connectInput(leftSub, rightSub);
    stereoSub.registerCallback(&FeatureTracker::Stereo_Callback, this);
    return true;
}

void FeatureTracker::Imu_Callback(const sensor_msgs::ImuConstPtr& imuMsg)
{
    if (state == FIRST_IMAGE) return;
    imuMsgBuffer.push_back(*imuMsg);
}

void FeatureTracker::Create_Image_Pyramid(void)
{
    leftPyramid.clear();
    rightPyramid.clear();
    const Mat& leftImageCurr = leftImagePtr->image;
    buildOpticalFlowPyramid( leftImageCurr, leftPyramid,
                             Size(WIN_SIZE, WIN_SIZE), PYRAMID_LEVEL, 
                             true, BORDER_REFLECT_101,
                             BORDER_CONSTANT, false );

    const Mat& rightImageCurr = rightImagePtr->image;
    buildOpticalFlowPyramid( rightImageCurr, rightPyramid,
                             Size(WIN_SIZE, WIN_SIZE), PYRAMID_LEVEL, 
                             true, BORDER_REFLECT_101,
                             BORDER_CONSTANT, false );
}

void FeatureTracker::Undistorted_Points(const vector<Point2f> ptsIn, vector<Point2f>& ptsOut, const Vec4d& intrinsics, 
                                        const Vec4d& distortionCoeffs, const Matx33d& rotation, const Vec4d& newIntrinsics)
{
    if ( ptsIn.size() == 0 ) return;
    
    const Matx33d k(intrinsics[0],           0.0, intrinsics[2],
                              0.0, intrinsics[1], intrinsics[3],
                              0.0,           0.0,           1.0);

    const Matx33d kNew(newIntrinsics[0],              0.0, newIntrinsics[2],
                                    0.0, newIntrinsics[1], newIntrinsics[3],
                                    0.0,              0.0,             1.0);

    undistortPoints(ptsIn, ptsOut, k, distortionCoeffs, rotation, kNew);
    return;
}

void FeatureTracker::Distortion_Points(const vector<Point2f> ptsIn, vector<Point2f>& ptsOut, 
                                       const Vec4d& intrinsics, const Vec4d& distortionCoeffs)
{
    const Matx33d k(intrinsics[0],           0.0, intrinsics[2],
                              0.0, intrinsics[1], intrinsics[3],
                              0.0,           0.0,           1.0);
    
    vector<cv::Point3f> homogenousPts;
    convertPointsToHomogeneous(ptsIn, homogenousPts);
    projectPoints(homogenousPts, Vec3d::zeros(), Vec3d::zeros(), k, distortionCoeffs, ptsOut);
    return;
}

bool FeatureTracker::Point_In_Border(const Point2f& pt)
{
    int x = cvRound(pt.x);
    int y = cvRound(pt.y);
    return (BOARD_SIZE < x && x <= (camResolution[0] - BOARD_SIZE) && BOARD_SIZE < y && y <= (camResolution[1] - BOARD_SIZE) );
}

void FeatureTracker::Stereo_Match(const vector<Point2f>& leftPoints, vector<Point2f>& rightPoints, vector<unsigned char>& inlierMarkers)
{
    if( leftPoints.size() == 0 ) return;

    if( rightPoints.size() == 0 ) 
    {
        vector<Point2f> leftKpsUndistorted;
        Undistorted_Points(leftPoints, leftKpsUndistorted, leftIntrinsics, leftDistortionCoeffs, r_left2right);
        Distortion_Points(leftKpsUndistorted, rightPoints, rightIntrinsics, rightDistortionCoeffs);
    }
    calcOpticalFlowPyrLK(leftPyramid, rightPyramid, leftPoints, rightPoints, 
                         inlierMarkers, noArray(), Size(WIN_SIZE, WIN_SIZE), PYRAMID_LEVEL, 
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), OPTFLOW_USE_INITIAL_FLOW);

    for( int i = 0; i < rightPoints.size(); i++ )
    {
        if( inlierMarkers[i] && !Point_In_Border(rightPoints[i]) )
        {
            inlierMarkers[i] = 0;
        }
    }

    // 利用对极约束删除外点
    const Matx33d tLeft2RightHat (              0.0, -t_left2right[2],  t_left2right[1],
                                    t_left2right[2],              0.0, -t_left2right[0],
                                   -t_left2right[1],  t_left2right[0],              0.0);
    const Matx33d E = tLeft2RightHat * r_left2right;

    vector<Point2f> leftKpsUndistorted(0);
    vector<Point2f> rightKpsUndistorted(0);
    Undistorted_Points(leftPoints, leftKpsUndistorted, leftIntrinsics, leftDistortionCoeffs);
    Undistorted_Points(rightPoints, rightKpsUndistorted, rightIntrinsics, rightDistortionCoeffs);

    double normPixelUnit = 4.0 / 
        ( leftIntrinsics[0] + leftIntrinsics[1] +
          rightIntrinsics[0] + rightIntrinsics[1] );
    
    for( int i = 0; i < leftKpsUndistorted.size(); i++ )
    {
        if (inlierMarkers[i])
        {
            Vec3d pt0( leftKpsUndistorted[i].x, leftKpsUndistorted[i].y, 1);
            Vec3d pt1( rightKpsUndistorted[i].x, rightKpsUndistorted[i].y, 1);
            Vec3d epipolar = E * pt0;
            double error = fabs((pt1.t() * epipolar)[0]) / sqrt( epipolar[0]*epipolar[0] + epipolar[1]*epipolar[1]);
            if (error > STEREO_THRESHOLD * normPixelUnit)
            {
                inlierMarkers[i] = 0;
            }
        }
    }
    return;
}

void FeatureTracker::Add_Points(void)
{
    if ( leftKpsAdd.size() != rightKpsAdd.size() )
    {
        ROS_WARN("the input size of leftKpsAdd(%lu) and rightKpsAdd(%lu) does not match: " , leftKpsAdd.size(), rightKpsAdd.size());
    }

    for ( int i = 0; i < leftKpsAdd.size(); i++)
    {
        leftKpsCurr.push_back( leftKpsAdd[i] );
        rightKpsCurr.push_back( rightKpsAdd[i] );
        trackerID.push_back(-1);
        trackerCnt.push_back(1);
    }
}

bool FeatureTracker::Update_Tracker_ID(int i)
{
    if ( i < trackerID.size() )
    {
        if ( trackerID[i] == -1 )
        {
             trackerID[i] = n_id++;
        }
        return true;
    }
    else
        return false;
}

void FeatureTracker::Set_Mask(void)
{
    mask = Mat(camResolution[1], camResolution[0], CV_8UC1, Scalar(255));

    vector<pair<LONGTYPE, pair< pair<Point2f, Point2f>, LONGTYPE>>> cnt_pts_id;

    for (unsigned int i = 0; i < leftKpsCurr.size(); i++)
    {
        cnt_pts_id.push_back(make_pair(trackerCnt[i], make_pair(make_pair(leftKpsCurr[i], rightKpsCurr[i]), trackerID[i])));
    }
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [] (const pair<LONGTYPE, pair< pair<Point2f, Point2f>, LONGTYPE>>& a, const pair<LONGTYPE, pair< pair<Point2f, Point2f>, LONGTYPE>>& b)
    {
        return a.first > b.first;
    });

    leftKpsCurr.clear();
    rightKpsCurr.clear();
    trackerCnt.clear();
    trackerID.clear();

    for ( auto& it : cnt_pts_id)
    {
        if ( mask.at<unsigned char>(it.second.first.first) == 255 )
        {
            trackerCnt.push_back(it.first);                         //重新放入特征点
            trackerID.push_back(it.second.second);
            leftKpsCurr.push_back(it.second.first.first);
            rightKpsCurr.push_back(it.second.first.second);
            circle(mask, it.second.first.first, TRACKER_DIS, 0, -1);
        }
    }
}

void FeatureTracker::Tracker_Feature(void)
{
    Matx33f left_R_p_c;
    Matx33f right_R_p_c;
    
    Predict_Feature_With_IMU(leftKpsRef, leftKpsCurr, leftIntrinsics, left_R_p_c, right_R_p_c);
    // cout << "===============" << endl;
    // ROS_INFO_STREAM(left_R_p_c);
    // ROS_INFO_STREAM(right_R_p_c);
    // cout << "===============" << endl;
    // 追踪左目特征点并删除外点
    vector<unsigned char> trackerInlier(0);
    calcOpticalFlowPyrLK(leftPyramidRef, leftPyramid, leftKpsRef, leftKpsCurr, 
                         trackerInlier, noArray(),Size(WIN_SIZE, WIN_SIZE), PYRAMID_LEVEL, 
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), OPTFLOW_USE_INITIAL_FLOW);

    for( int i = 0; i < int(leftKpsCurr.size()); i++)
    {
        // ROS_INFO_STREAM("the position of feature: " << leftKpsCurr[i]);
        if ( trackerInlier[i] && !Point_In_Border(leftKpsCurr[i]) )
        {
            trackerInlier[i] = 0;
        }
    }
    Reduce_Vector(leftKpsRef, trackerInlier);
    Reduce_Vector(leftKpsCurr, trackerInlier);
    Reduce_Vector(rightKpsRef, trackerInlier);
    Reduce_Vector(trackerID, trackerInlier);
    Reduce_Vector(trackerCnt, trackerInlier);

    // 双目特征点匹配并删除外点
    vector<unsigned char> inlierMarkers(0);
    Stereo_Match(leftKpsCurr, rightKpsCurr, inlierMarkers);
    
    // ROS_INFO_STREAM("the number of features before matching: " << leftKpsCurr.size());
    Reduce_Vector(leftKpsRef, inlierMarkers);
    Reduce_Vector(leftKpsCurr, inlierMarkers);
    Reduce_Vector(rightKpsRef, inlierMarkers);
    Reduce_Vector(rightKpsCurr, inlierMarkers);
    Reduce_Vector(trackerID, inlierMarkers);
    Reduce_Vector(trackerCnt, inlierMarkers);
    // ROS_INFO_STREAM("the number of features after matching: " << leftKpsCurr.size());
    
    vector<unsigned char> leftInlierMarkers(0);
    Delet_Point_With_RANSAC(leftKpsRef, leftKpsCurr, left_R_p_c, leftIntrinsics, leftDistortionCoeffs, 0.99, leftInlierMarkers);
    
    vector<unsigned char> rightInlierMarkers(0);
    Delet_Point_With_RANSAC(rightKpsRef, rightKpsCurr, right_R_p_c, rightIntrinsics, rightDistortionCoeffs, 0.99, rightInlierMarkers);

    vector<unsigned char> ransacMarker(leftInlierMarkers.size(), 0);
    // if (leftInlierMarkers.size() != rightInlierMarkers.size())
    // {
    //     ROS_WARN("inlier markers do not match in RANSAC");
    // }
    for (int i = 0; i < leftInlierMarkers.size(); i++)
    {
        // cout << "left: " << (int)(leftInlierMarkers[i]) << " right: " << (int)(rightInlierMarkers[i]) << endl;
        if (leftInlierMarkers[i] == 1 && rightInlierMarkers[i] == 1)
        {
            ransacMarker[i] = 1;
        }
        else
        {
            ransacMarker[i] = 0;
        }
    }
    // float n1 = leftKpsCurr.size();
    // ROS_INFO_STREAM("----------------------------");
    // ROS_INFO_STREAM("the number of features before matching: " << leftKpsCurr.size());
    Reduce_Vector(leftKpsRef, ransacMarker);
    Reduce_Vector(leftKpsCurr, ransacMarker);
    Reduce_Vector(rightKpsRef, ransacMarker);
    Reduce_Vector(rightKpsCurr, ransacMarker);
    Reduce_Vector(trackerID, ransacMarker);
    Reduce_Vector(trackerCnt, ransacMarker);
    // float n2 = leftKpsCurr.size();
    // ROS_INFO_STREAM("the number of features after ransac: " << n2);
    // ROS_INFO_STREAM("the ratio of features after ransac: " << n2 / n1);
    // ROS_INFO_STREAM("the number of features after matching: " << leftKpsCurr.size());
}
void FeatureTracker::Delet_Point_With_RANSAC(vector<Point2f>& pts1, vector<Point2f>& pts2, const cv::Matx33f& R_p_c, const cv::Vec4d& intrinsics, const cv::Vec4d& distortionCoeffs, const float& success, vector<unsigned char>& inlierMarkers)
{
    if(pts1.size() != pts2.size())
    {
        ROS_ERROR("Sets of different size (%lu and %lu) are used in RANSAC", pts1.size(), pts2.size());
    }
    //　归一化像素点
    double normPixelUnit = 2.0 / (intrinsics[0]+intrinsics[1]);
    // ROS_INFO_STREAM("normal pixel unit1: " << normPixelUnit);

    inlierMarkers.clear();
    inlierMarkers.resize(pts1.size(), 1);

    vector<Point2f> pts1Undistorted(pts1.size());
    vector<Point2f> pts2Undistorted(pts2.size());
    Undistorted_Points(pts1, pts1Undistorted, intrinsics, distortionCoeffs);
    Undistorted_Points(pts2, pts2Undistorted, intrinsics, distortionCoeffs);

    //　补偿前一帧点，使其只有平移
    for (auto& pt : pts1Undistorted)
    {
        Vec3f pt1(pt.x, pt.y, 1.0f);
        Vec3f pt2 = R_p_c * pt1;
        pt.x = pt2[0];
        pt.y = pt2[1];
    }

    // 归一化所有坐标,转化为无量纲形式
    float scale = 0;

    for (int i = 0; i < pts1Undistorted.size(); i++)
    {
        scale += sqrt(pts1Undistorted[i].dot(pts1Undistorted[i]));    //　这样做是为了避免里面有负数
        scale += sqrt(pts2Undistorted[i].dot(pts2Undistorted[i]));
    }
    scale = (pts1Undistorted.size() + pts2Undistorted.size()) / scale /** sqrt(2.0f)*/;

    for (int i = 0; i < pts1Undistorted.size(); i++)
    {
        pts1Undistorted[i] *= scale;
        pts2Undistorted[i] *= scale;
    }
    normPixelUnit *= scale;
    // ROS_INFO_STREAM("normal pixel unit2: " << normPixelUnit);

    // 求两个像素点之间的位移：　上次　－　当前
    vector<Point2f> ptsDiff(pts1Undistorted.size());
    for (int i = 0; i < pts1Undistorted.size(); i++)
    {
        ptsDiff[i] = pts1Undistorted[i] - pts2Undistorted[i];
    }

    double meanDist = 0;
    int inlierCnt = 0;
    for (int i = 0; i < ptsDiff.size();i++)
    {
        double distance = sqrt(ptsDiff[i].dot(ptsDiff[i]));
        // ROS_INFO_STREAM("distance: " << distance);
        if (distance > 50 * normPixelUnit)
        {
            inlierMarkers[i] = 0;
        }
        else
        {
            inlierCnt++;
            meanDist += distance;
        }
    }
    meanDist /= inlierCnt;
    // ROS_INFO_STREAM("mean distance: " << meanDist);

    //　如果内点小于３个，则将所有点标记为外点
    if (inlierCnt < 3) 
    {
        for (auto& marker : inlierMarkers) marker = 0;
        return;
    }

    //　判断是否没有平移
    if (meanDist < normPixelUnit) 
    {
        for (int i = 0; i < ptsDiff.size(); i++) 
        {
            if (inlierMarkers[i] == 0) continue;
            // ROS_INFO_STREAM("mean distance: " << meanDist);
            // ROS_INFO_STREAM("norm pixelUnit: " << normPixelUnit);
            // ROS_INFO_STREAM("ptsDiff: " << sqrt(ptsDiff[i].dot(ptsDiff[i])));
            // ROS_INFO_STREAM("trackerID: " << trackerID[i]);
            if (sqrt(ptsDiff[i].dot(ptsDiff[i])) > RANSAC_THRESHOLD*normPixelUnit)
            {
                inlierMarkers[i] = 0;
                // ROS_INFO_STREAM("Yes");
            }

        }
        return;
    }

    int iterNumber = static_cast<int> (ceil(log(1 - success) / log(1 - 0.7 * 0.7)));
    vector<int> rawInlierID;
    for (int i = 0; i < inlierMarkers.size(); i++)
    {
        if (inlierMarkers[i])
        {
            rawInlierID.push_back(i);
        }
    }

    Eigen::MatrixXd equationT(ptsDiff.size(), 3);
    for (int i = 0; i < ptsDiff.size(); ++i) 
    {
        equationT(i, 0) = ptsDiff[i].y;
        equationT(i, 1) = -ptsDiff[i].x;
        equationT(i, 2) = pts1Undistorted[i].x * pts2Undistorted[i].y - pts1Undistorted[i].y * pts2Undistorted[i].x;
    }

    vector<int> bestInlier;
    double modelError = 1e10;
    random_numbers::RandomNumberGenerator randomGen;

    for(int iterCnt = 0; iterCnt < iterNumber; iterCnt++)
    {
        int selectID1 = randomGen.uniformInteger(0, rawInlierID.size()-1);
        int selectDistance = randomGen.uniformInteger(1, rawInlierID.size()-1);
        int selectID2 = (selectID1 + selectDistance) < rawInlierID.size() ? (selectID1 + selectDistance) : (selectID1 + selectDistance - rawInlierID.size());

        int selectPoint1 = rawInlierID[selectID1];
        int selectPoint2 = rawInlierID[selectID2];

        Eigen::Vector2d equation_tx(equationT(selectPoint1, 0), equationT(selectPoint2, 0));
        Eigen::Vector2d equation_ty(equationT(selectPoint1, 1), equationT(selectPoint2, 1));
        Eigen::Vector2d equation_tz(equationT(selectPoint1, 2), equationT(selectPoint2, 2));
        vector<double> equationNorm(3);
        equationNorm[0] = equation_tx.lpNorm<1>();
        equationNorm[1] = equation_ty.lpNorm<1>();
        equationNorm[2] = equation_tz.lpNorm<1>();
        int indicator = min_element(equationNorm.begin(), equationNorm.end()) - equationNorm.begin();

        Eigen::Vector3d modelParam(0.0, 0.0, 0.0);
        if (indicator == 0) 
        {
            Eigen::Matrix2d A;
            A << equation_ty, equation_tz;
            Eigen::Vector2d solution = A.inverse() * (-equation_tx);
            modelParam(0) = 1.0;
            modelParam(1) = solution(0);
            modelParam(2) = solution(1);
        } 
        else if (indicator ==1) 
        {
            Eigen::Matrix2d A;
            A << equation_tx, equation_tz;
            Eigen::Vector2d solution = A.inverse() * (-equation_ty);
            modelParam(0) = solution(0);
            modelParam(1) = 1.0;
            modelParam(2) = solution(1);
        } 
        else 
        {
            Eigen::Matrix2d A;
            A << equation_tx, equation_ty;
            Eigen::Vector2d solution = A.inverse() * (-equation_tz);
            modelParam(0) = solution(0);
            modelParam(1) = solution(1);
            modelParam(2) = 1.0;
        }

        Eigen::VectorXd err = equationT * modelParam;

        vector<int> inlierID;
        for (int i = 0; i < err.rows(); i++)
        {
            if (inlierMarkers[i] == 0) continue;
            if (abs(err[i]) < RANSAC_THRESHOLD*normPixelUnit)
            {
                inlierID.push_back(i);
            }
        }

        // the model is bad. reject!
        if (inlierID.size() < 0.2*pts1Undistorted.size()) continue;

        Eigen::VectorXd equationBetter_tx(inlierID.size());
        Eigen::VectorXd equationBetter_ty(inlierID.size());
        Eigen::VectorXd equationBetter_tz(inlierID.size());

        for (int i = 0; i < inlierID.size(); ++i) 
        {
            equationBetter_tx(i) = equationT(inlierID[i], 0);
            equationBetter_ty(i) = equationT(inlierID[i], 1);
            equationBetter_tz(i) = equationT(inlierID[i], 2);
        }

        Eigen::Vector3d modelParamBetter(0.0, 0.0, 0.0);
        if (indicator == 0) 
        {
            Eigen::MatrixXd A(inlierID.size(), 2);
            A << equationBetter_ty, equationBetter_tz;
            Eigen::Vector2d solution = (A.transpose() * A).inverse() * A.transpose() * (-equationBetter_tx);
            modelParamBetter(0) = 1.0;
            modelParamBetter(1) = solution(0);
            modelParamBetter(2) = solution(1);
        } 
        else if (indicator == 1) 
        {
            Eigen::MatrixXd A(inlierID.size(), 2);
            A << equationBetter_tx, equationBetter_tz;
            Eigen::Vector2d solution = (A.transpose() * A).inverse() * A.transpose() * (-equationBetter_ty);
            modelParamBetter(0) = solution(0);
            modelParamBetter(1) = 1.0;
            modelParamBetter(2) = solution(1);
        } 
        else 
        {
            Eigen::MatrixXd A(inlierID.size(), 2);
            A << equationBetter_tx, equationBetter_ty;
            Eigen::Vector2d solution = (A.transpose() * A).inverse() * A.transpose() * (-equationBetter_tz);
            modelParamBetter(0) = solution(0);
            modelParamBetter(1) = solution(1);
            modelParamBetter(2) = 1.0;
        }

        Eigen::VectorXd newErr = equationT * modelParamBetter;
        
        double meanError = 0;
        for (const auto& id : inlierID)
        {
            meanError += abs(newErr(id));
        }
        meanError /= inlierID.size();

        if( inlierID.size() > bestInlier.size())
        {
            modelError = meanError;
            bestInlier = inlierID;
        }
    }

    //  fill the inlierMarkers vector.
    inlierMarkers.clear();
    inlierMarkers.resize(pts1.size(), 0);
    for (const auto& id : bestInlier)
    {
        inlierMarkers[id] = 1;
    }
}



// void FeatureTracker::Delet_Point_With_F(void)
// {
//     if (leftKpsCurr.size() >= 8 && rightKpsCurr.size() >= 8)
//     {
//         vector<Point2f> leftUndistortedRef;
//         vector<Point2f> leftUndistortedCurr;
//         vector<Point2f> rightUndistortedRef;
//         vector<Point2f> rightUndistortedCurr;
//         Undistorted_Points(leftKpsRef, leftUndistortedRef, leftIntrinsics, leftDistortionCoeffs);
//         Undistorted_Points(leftKpsCurr, leftUndistortedCurr, leftIntrinsics, leftDistortionCoeffs);
//         Undistorted_Points(rightKpsRef, rightUndistortedRef, rightIntrinsics, rightDistortionCoeffs);
//         Undistorted_Points(rightKpsCurr, rightUndistortedCurr, rightIntrinsics, rightDistortionCoeffs);

//         vector<unsigned char> leftInlierMarkers(0);
//         findFundamentalMat(leftUndistortedRef, leftUndistortedCurr, FM_RANSAC, F_THRESHOLD, 0.99, leftInlierMarkers);

//         vector<unsigned char> rightInlierMarkers(0);
//         findFundamentalMat(rightUndistortedRef, rightUndistortedCurr, FM_RANSAC, F_THRESHOLD, 0.99, rightInlierMarkers);

//         if ( leftInlierMarkers.size() != rightInlierMarkers.size())
//         {
//             ROS_WARN("the leftInlierMarkers(%lu) and the rightInlierMarkers(%lu) does not match!", leftInlierMarkers.size(), rightInlierMarkers.size());
//         }
//         vector<unsigned char> inlierMarkers(leftInlierMarkers.size(), 0);
//         for (int i = 0; i < leftInlierMarkers.size(); i++)
//         {
//             if ( leftInlierMarkers[i] == 1 && rightInlierMarkers[i] == 1)
//             {
//                 inlierMarkers[i] = 1;
//             }
//         }
//         // ROS_INFO_STREAM("the number of features before delet point with f: " << leftKpsCurr.size());
//         Reduce_Vector(leftKpsRef, inlierMarkers);
//         Reduce_Vector(leftKpsCurr, inlierMarkers);
//         Reduce_Vector(rightKpsRef, inlierMarkers);
//         Reduce_Vector(rightKpsCurr, inlierMarkers);
//         Reduce_Vector(trackerID, inlierMarkers);
//         Reduce_Vector(trackerCnt, inlierMarkers);
//         // ROS_INFO_STREAM("the number of features after delet point with f: " << leftKpsCurr.size());
//     }
// }

void FeatureTracker::Predict_Feature_With_IMU(vector<Point2f>& ptsIn, vector<Point2f>& ptsOut, const Vec4d& intrinsics, Matx33f& left_R_p_c, Matx33f& right_R_p_c)
{
    auto beginIter = imuMsgBuffer.begin();
    while(beginIter != imuMsgBuffer.end())
    {
        if (beginIter->header.stamp.toSec() < lastImageTime)
        {
            beginIter++;
        }
        else
            break;
    }
    auto endIter = beginIter;
    while(endIter != imuMsgBuffer.end())
    {
        if(endIter->header.stamp.toSec() < currImageTime )
        {
            endIter++;
        }
        else
            break;
    }
    // ROS_INFO_STREAM((endIter - beginIter));
    Vec3f meanAngleVel(0.0, 0.0, 0.0);
    for (auto iter = beginIter; iter < endIter; iter++)
    {
        meanAngleVel += Vec3f(iter->angular_velocity.x, iter->angular_velocity.y, iter->angular_velocity.z);
    }
    if (endIter - beginIter > 0)
    {
        meanAngleVel *= 1.0f / (endIter - beginIter);
    }

    Vec3f leftMeanAngleVel = r_left2imu.t() * meanAngleVel;
    Vec3f rightMeanAngleVel = r_right2imu.t() * meanAngleVel;

    double dTime = currImageTime - lastImageTime;
    // ROS_INFO_STREAM(meanAngleVel * dTime);
    Rodrigues(leftMeanAngleVel*dTime, left_R_p_c);
    Rodrigues(rightMeanAngleVel*dTime, right_R_p_c);
    left_R_p_c = left_R_p_c.t();
    right_R_p_c = right_R_p_c.t();
    // erase data
    imuMsgBuffer.erase(imuMsgBuffer.begin(), endIter);

    ptsOut.resize(ptsIn.size());
    const Matx33f k(intrinsics[0],           0.0, intrinsics[2],
                              0.0, intrinsics[1], intrinsics[3],
                              0.0,           0.0,           1.0);
    Matx33f H = k * left_R_p_c * k.inv();
    for (int i = 0; i < ptsIn.size(); i++)
    {
        Vec3f p1(ptsIn[i].x, ptsIn[i].y, 1.0f);
        Vec3f p2 = H * p1;
        ptsOut[i].x = p2[0] / p2[2];
        ptsOut[i].y = p2[1] / p2[2];
    }
}

void FeatureTracker::Find_Image_Feature(void)
{
    const Mat& leftImgCurr = leftImagePtr->image;

    leftKpsCurr.clear();
    rightKpsCurr.clear();
    cameraKps3d.clear();

    // 若左右目上一次特征点不为空，开始追踪特征点
    if (!leftKpsRef.empty() && !rightKpsRef.empty())
    {
        Tracker_Feature();
        // Delet_Point_With_F();
    }
    for ( auto& num : trackerCnt)           
        num++;
    
    // 如果需要发布数据，则删除帧间误匹配，增补新的特征点
    if ( pubThisFrame )
    {
        Set_Mask();
        leftKpsAdd.clear();
        rightKpsAdd.clear();
        // ROS_INFO_STREAM("the max number of feature: " << TRACKER_NUM);
        int featureMaxCnt = TRACKER_NUM - static_cast<int>(leftKpsCurr.size());
        if ( featureMaxCnt > 0)
        {
            if(mask.empty())
                ROS_INFO_STREAM("mask is empty ");
            if (mask.type() != CV_8UC1)
                ROS_INFO_STREAM("mask type wrong ");
            if (mask.size() != leftImgCurr.size())
                ROS_INFO_STREAM("mask wrong size " << mask.size() << "image size: " << leftImgCurr.size());
            goodFeaturesToTrack(leftImgCurr, leftKpsAdd, featureMaxCnt, 0.05, TRACKER_DIS, mask);
            cornerSubPix(leftImgCurr, leftKpsAdd, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.001));
            // for (int i = 1; i < leftKpsAdd.size(); i++)
            // {
            //     ROS_INFO_STREAM("the position of feature: " << leftKpsAdd[i]);
            // }
            vector<unsigned char> inlierMarkers(0);
            Stereo_Match(leftKpsAdd, rightKpsAdd, inlierMarkers);

            Reduce_Vector(leftKpsAdd, inlierMarkers);
            Reduce_Vector(rightKpsAdd, inlierMarkers);
        }
        Add_Points(); 

        vector<Point2f> leftKpsUndistorted(0);
        vector<Point2f> rightKpsUndistorted(0);
        Undistorted_Points(leftKpsCurr, leftKpsUndistorted, leftIntrinsics, leftDistortionCoeffs);
        Undistorted_Points(rightKpsCurr, rightKpsUndistorted, rightIntrinsics, rightDistortionCoeffs);
    
        Triangulate_Points(leftKpsUndistorted, rightKpsUndistorted);
        vector<unsigned char> inlierMarkers(0);
        inlierMarkers.resize(cameraKps3d.size(), 1);
        for( int i = 0; i < cameraKps3d.size(); i++)
        {
            if (cameraKps3d[i].z <= 0)
            {
                inlierMarkers[i] = 0;
            }
        }
        Reduce_Vector(leftKpsCurr, inlierMarkers);
        Reduce_Vector(rightKpsCurr, inlierMarkers);
        Reduce_Vector(trackerID, inlierMarkers);
        Reduce_Vector(trackerCnt, inlierMarkers);
        Reduce_Vector(cameraKps3d, inlierMarkers);
        
        inlierMarkers.resize(cameraKps3d.size(), 1);
        float meanDistance = 0;
        for (int i = 0; i < cameraKps3d.size(); i++)
        {
            meanDistance += cameraKps3d[i].z;
        }
        meanDistance /= cameraKps3d.size();
        // ROS_INFO_STREAM(meanDistance); 
        for (int i = 0; i < cameraKps3d.size(); i++)
        {
            if(cameraKps3d[i].z > 4 * meanDistance)
            {
                inlierMarkers[i] = 0;
            }
        }
        Reduce_Vector(leftKpsCurr, inlierMarkers);
        Reduce_Vector(rightKpsCurr, inlierMarkers);
        Reduce_Vector(trackerID, inlierMarkers);
        Reduce_Vector(trackerCnt, inlierMarkers);
        Reduce_Vector(cameraKps3d, inlierMarkers);
    }
    
    for( unsigned int i = 0;; i++ )
    {
        bool completed = false;
        completed |= Update_Tracker_ID(i);
        if (!completed)
            break;
    }

    swap(leftPyramidRef, leftPyramid);
    // leftPyramidRef = leftPyramid;
    leftKpsRef = leftKpsCurr;
    rightKpsRef = rightKpsCurr;
}

void FeatureTracker::Publish_Info(void)
{
    feature_tracker::CameraTrackerResultPtr featurePoints(new feature_tracker::CameraTrackerResult);
    featurePoints->header.stamp = leftImagePtr->header.stamp;
    featurePoints->header.frame_id = "camera_normalized";

    vector<Point2f> leftKpsUndistorted(0);
    vector<Point2f> rightKpsUndistorted(0);
    Undistorted_Points(leftKpsCurr, leftKpsUndistorted, leftIntrinsics, leftDistortionCoeffs);
    Undistorted_Points(rightKpsCurr, rightKpsUndistorted, rightIntrinsics, rightDistortionCoeffs);
    
    // Triangulate_Points(leftKpsUndistorted, rightKpsUndistorted);

    int featureNum = 0;
    for (int i = 0; i < trackerID.size(); i++)
    {
        featureNum++;
        featurePoints->features.push_back(feature_tracker::FeatureTrackerResult());
        featurePoints->features[i].id = trackerID[i];
        featurePoints->features[i].cnt = trackerCnt[i];
        featurePoints->features[i].seq = featureNum;
        featurePoints->features[i].u0 = leftKpsUndistorted[i].x;
        featurePoints->features[i].v0 = leftKpsUndistorted[i].y;
        featurePoints->features[i].u1 = rightKpsUndistorted[i].x;
        featurePoints->features[i].v1 = rightKpsUndistorted[i].y; 
        featurePoints->features[i].x = cameraKps3d[i].x;
        featurePoints->features[i].y = cameraKps3d[i].y;
        featurePoints->features[i].z = cameraKps3d[i].z;
    }
    featurePoints->num_of_features = featureNum;
    pubFeatures.publish(featurePoints);

    if (pubMatchImage.getNumSubscribers() > 0 && SHOW_TRACKER)
    {
        Mat outImg(camResolution[1], camResolution[0]*2, CV_8UC3);
        cvtColor(leftShow, outImg.colRange(0, camResolution[0]), CV_GRAY2RGB);
        cvtColor(rightShow, outImg.colRange(camResolution[0], camResolution[0]*2), CV_GRAY2RGB);

        if(lastLeftMap.size() != 0)
        {
            for(int i = 0; i < trackerID.size(); i++)
            {
                if(lastLeftMap.find(trackerID[i]) != lastLeftMap.end() && lastRightMap.find(trackerID[i]) != lastRightMap.end())
                {
                    Point2f prev_pt0 = lastLeftMap[trackerID[i]];
                    Point2f prev_pt1 = lastRightMap[trackerID[i]] + Point2f(camResolution[0], 0.0);
                    Point2f curr_pt0 = leftKpsCurr[i];
                    Point2f curr_pt1 = rightKpsCurr[i] + Point2f(camResolution[0], 0.0);
                    double cnt = min(1.0, 1.0*trackerCnt[i]/TRACKER_SIZE);
                    // circle(outImg, curr_pt0, 8, Scalar(0, 255*(1-cnt), 255*cnt), -1);
                    // circle(outImg, curr_pt1, 8, Scalar(0, 255*(1-cnt), 255*cnt), -1);
                    // // line(outImg, prev_pt0, curr_pt0, Scalar(0, 225, 255), 4, LINE_AA);
                    // arrowedLine(outImg, prev_pt0, curr_pt0, Scalar(0, 255, 255), 2, LINE_AA, 0, 0.3);
                    // arrowedLine(outImg, prev_pt1, curr_pt1, Scalar(0, 255, 255), 2, LINE_AA, 0, 0.3);
                    // string textShow = to_string(static_cast<int>(cameraKps3d[i].z));
                    // int baseLine;
                    // Size textSize = getTextSize(textShow, FONT_HERSHEY_SIMPLEX, 1, 1, &baseLine);
                    // putText(outImg, textShow, leftKpsCurr[i], FONT_HERSHEY_SIMPLEX, 1,  Scalar(255, 0, 0), 1, LINE_AA);

                    circle(outImg, curr_pt0, 3, Scalar(0, 255*(1-cnt), 255*cnt), -1);
                    circle(outImg, curr_pt1, 3, Scalar(0, 255*(1-cnt), 255*cnt), -1);
                    // line(outImg, prev_pt0, curr_pt0, Scalar(0, 225, 255), 4, LINE_AA);
                    arrowedLine(outImg, prev_pt0, curr_pt0, Scalar(0, 255, 255), 1, LINE_AA, 0, 0.2);
                    arrowedLine(outImg, prev_pt1, curr_pt1, Scalar(0, 255, 255), 1, LINE_AA, 0, 0.2);
                    string textShow = to_string(static_cast<int>(cameraKps3d[i].z));
                    int baseLine;
                    Size textSize = getTextSize(textShow, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                    putText(outImg, textShow, leftKpsCurr[i], FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(255, 0, 0), 1, LINE_AA);
                    lastLeftMap.erase(trackerID[i]);
                    lastRightMap.erase(trackerID[i]);
                }
            }
        }

        lastLeftMap.clear();
        lastRightMap.clear();
        for (int i = 0; i < trackerID.size(); i++)
        {
            lastLeftMap[trackerID[i]] = leftKpsCurr[i];
            lastRightMap[trackerID[i]] = rightKpsCurr[i];
        }

        // for( unsigned int i = 0; i < trackerID.size(); i++)
        // {
        //     if(trackerCnt[i] != 1)
        //     {
        //         double cnt = min(1.0, 1.0*trackerCnt[i]/TRACKER_SIZE);
        //         circle(outImg, leftKpsCurr[i], 3, Scalar(0, 255*(1-cnt), 255*cnt), -1);
        //         circle(outImg, rightKpsCurr[i]+ Point2f(camResolution[0], 0.0), 3, Scalar(0, 255*(1-cnt), 255*cnt), -1);
        //         // line(outImg, leftKpsCurr[i], rightKpsCurr[i]+ Point2f(camResolution[0], 0.0), Scalar(0, 225, 255), 1, LINE_AA);
        //         string textShow = to_string(static_cast<int>(trackerID[i]));
        //         int baseLine;
        //         Size textSize = getTextSize(textShow, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        //         putText(outImg, textShow, leftKpsCurr[i], FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(255, 0, 0), 1, LINE_AA);
        //     }
        // }
        cv_bridge::CvImage debug_image(leftImagePtr->header, "bgr8", outImg);
        pubMatchImage.publish(debug_image.toImageMsg());
    }
}

void FeatureTracker::Triangulate_Points(vector<Point2f> leftPts, vector<Point2f> rightPts)  // 相机坐标系下无畸变归一化点
{
    Mat t1 = (Mat_<float>(3, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0 ,1 ,0);
    Mat t2 = (Mat_<float>(3, 4) << 
    T_left2right.at<double>(0, 0), T_left2right.at<double>(0, 1), T_left2right.at<double>(0, 2), T_left2right.at<double>(0, 3),
    T_left2right.at<double>(1, 0), T_left2right.at<double>(1, 1), T_left2right.at<double>(1, 2), T_left2right.at<double>(1, 3),
    T_left2right.at<double>(2, 0), T_left2right.at<double>(2, 1), T_left2right.at<double>(2, 2), T_left2right.at<double>(2, 3));

    Mat pts4d;
    if (leftPts.size() == 0 || rightPts.size() == 0)
    {
        ROS_INFO_STREAM("There is no feature! ");
        return;
    }
        
    triangulatePoints(t1, t2, leftPts, rightPts, pts4d);

    // 归一化为3d点
    for (int i = 0; i < pts4d.cols; i++)
    {
        Mat x = pts4d.col(i);
        x /= x.at<float>(3, 0);
        Point3f p (
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0));

        cameraKps3d.push_back( p );
        
    }
}

void FeatureTracker::Stereo_Callback(const sensor_msgs::ImageConstPtr& leftImg, const sensor_msgs::ImageConstPtr& rightImg)
{
    static char imageCnt = 0;
    uint64 timeBegin = ros::Time::now().toNSec();
    // ROS_INFO_STREAM("The image time: " << leftImg->header.stamp.toSec());
    if ( state == FIRST_IMAGE )
    {
        if(++imageCnt > 5)
        {
            state = NOT_FIRST_IMAGE;        // 状态转换
        }
        currImageTime = firstImageTime = leftImg->header.stamp.toSec();
        lastImageTime = currImageTime;
        return;
    }
    if ( (leftImg->header.stamp.toSec()-currImageTime) > 1.0 )
    {
        ROS_WARN("image fault! reset the feature tracker!");
        leftKpsRef.clear();
        rightKpsRef.clear();
        state = FIRST_IMAGE;
        pubCnt = 1;
        return;
    }
    currImageTime = leftImg->header.stamp.toSec();  //获取当前帧时间戳
    
    // 控制频率
    if ( round(1.0*pubCnt / (currImageTime - firstImageTime)) <= FREQ )
    {   
        pubThisFrame = true;                        //发布标志位置位

        if ( abs(1.0*pubCnt/(currImageTime - firstImageTime) - FREQ) < 0.02 * FREQ)   // 频率控制误差
        {
            firstImageTime = currImageTime;         // 重置图像帧数时间戳
            pubCnt = 0;                             // 重置发布计数器
        }
    }
    else
    {
        pubThisFrame = false;
    }
    
    leftImagePtr = cv_bridge::toCvCopy(leftImg, enc::MONO8);
    rightImagePtr = cv_bridge::toCvCopy(rightImg, enc::MONO8);
    
    if (pubMatchImage.getNumSubscribers() > 0 && SHOW_TRACKER)
    {
        leftShow = leftImagePtr->image.clone();
        rightShow = rightImagePtr->image.clone();
    }


    if ( EQUALIZE )
    {
        Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
        clahe->apply(leftImagePtr->image, leftImagePtr->image);
        clahe->apply(rightImagePtr->image, rightImagePtr->image);
    }


    Create_Image_Pyramid();

    Find_Image_Feature();

    if( pubThisFrame )
    {
        pubCnt++;
        Publish_Info();
    }
    lastImageTime = currImageTime;
    uint64 timeEnd = ros::Time::now().toNSec();
    // ROS_INFO_STREAM("code cost time: " << (timeEnd - timeBegin) << " ns");
}

}
