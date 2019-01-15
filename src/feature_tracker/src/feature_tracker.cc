#include <feature_tracker/feature_tracker.h>

namespace slam_mono
{
FeatureTracker::FeatureTracker(ros::NodeHandle& nh):n(nh), state(FIRST_IMAGE), pubCnt(1), pubThisFrame(false), firstImageTime(0), currImageTime(0), stereoSub(1)
{
    if( !Load_Parameters() ) return;
    ROS_INFO(" load parameters success! ");

    if ( !Create_RosIO() ) return;
    ROS_INFO(" create ROS IO finished! ");
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

    n.param<int>("frequence", FREQ, 10);
    n.param<int>("board_size", BOARD_SIZE, 1);
    n.param<int>("tracker_size", TRACKER_SIZE, 30);
    n.param<int>("tracker_min_distance", TRACKER_DIS, 20);
    n.param<int>("tracker_max_count", TRACKER_NUM, 100);
    n.param<int>("pyramid_level", PYRAMID_LEVEL, 3);
    n.param<int>("win_size", WIN_SIZE, 15);
    n.param<double>("f_threshold", F_THRESHOLD, 1.0);
    n.getParam("left/topic", LEFT_TOPICS);
    n.getParam("right/topic", RIGHT_TOPICS);
    n.getParam("pub_match_image_topic", FEATURE_IMAGE_TOPICS);
    n.getParam("pub_feature_topic", FEATURE_TOPICS);
    n.getParam("equalize", EQUALIZE);
    

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
    ROS_INFO("f_threshold: %f", F_THRESHOLD);
    cout << "left_topics: " << LEFT_TOPICS << endl;
    cout << "right_topics: " << RIGHT_TOPICS << endl;
    cout << "pub_match_image_topic: " << FEATURE_IMAGE_TOPICS << endl;
    cout << "pub_feature_topic: " << FEATURE_TOPICS << endl;
    ROS_INFO("====================================================");
    return true;
}

bool FeatureTracker::Create_RosIO(void)
{
    image_transport::ImageTransport it(n);
    pubMatchImage = it.advertise(FEATURE_IMAGE_TOPICS, 1);
    pubFeatures = n.advertise<feature_tracker::CameraTrackerResult>(FEATURE_TOPICS, 1);
    leftSub.subscribe(n, LEFT_TOPICS, 1);
    rightSub.subscribe(n, RIGHT_TOPICS, 1);
    stereoSub.connectInput(leftSub, rightSub);
    stereoSub.registerCallback(&FeatureTracker::Stereo_Callback, this);
    return true;
}

void FeatureTracker::Create_Image_Pyramid(void)
{
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
            if (error > 5 * normPixelUnit)
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
    // 追踪左目特征点并删除外点
    vector<unsigned char> trackerInlier(0);
    calcOpticalFlowPyrLK(leftPyramidRef, leftPyramid, leftKpsRef, leftKpsCurr, 
                         trackerInlier, noArray(),Size(WIN_SIZE, WIN_SIZE), PYRAMID_LEVEL, 
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01));
    
    for( int i = 0; i < int(leftKpsCurr.size()); i++)
    {
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
    
    ROS_INFO_STREAM("the number of features before matching: " << leftKpsCurr.size());
    Reduce_Vector(leftKpsRef, inlierMarkers);
    Reduce_Vector(leftKpsCurr, inlierMarkers);
    Reduce_Vector(rightKpsRef, inlierMarkers);
    Reduce_Vector(rightKpsCurr, inlierMarkers);
    Reduce_Vector(trackerID, inlierMarkers);
    Reduce_Vector(trackerCnt, inlierMarkers);
    ROS_INFO_STREAM("the number of features after matching: " << leftKpsCurr.size());
}

void FeatureTracker::Delet_Point_With_F(void)
{
    if (leftKpsCurr.size() >= 8 && rightKpsCurr.size() >= 8)
    {
        vector<Point2f> leftUndistortedRef;
        vector<Point2f> leftUndistortedCurr;
        vector<Point2f> rightUndistortedRef;
        vector<Point2f> rightUndistortedCurr;
        Undistorted_Points(leftKpsRef, leftUndistortedRef, leftIntrinsics, leftDistortionCoeffs);
        Undistorted_Points(leftKpsCurr, leftUndistortedCurr, leftIntrinsics, leftDistortionCoeffs);
        Undistorted_Points(rightKpsRef, rightUndistortedRef, rightIntrinsics, rightDistortionCoeffs);
        Undistorted_Points(rightKpsCurr, rightUndistortedCurr, rightIntrinsics, rightDistortionCoeffs);

        vector<unsigned char> leftInlierMarkers(0);
        findFundamentalMat(leftUndistortedRef, leftUndistortedCurr, FM_RANSAC, F_THRESHOLD, 0.99, leftInlierMarkers);

        vector<unsigned char> rightInlierMarkers(0);
        findFundamentalMat(rightUndistortedRef, rightUndistortedCurr, FM_RANSAC, F_THRESHOLD, 0.99, rightInlierMarkers);

        if ( leftInlierMarkers.size() != rightInlierMarkers.size())
        {
            ROS_WARN("the leftInlierMarkers(%lu) and the rightInlierMarkers(%lu) does not match!", leftInlierMarkers.size(), rightInlierMarkers.size());
        }
        vector<unsigned char> inlierMarkers(leftInlierMarkers.size(), 0);
        for (int i = 0; i < leftInlierMarkers.size(); i++)
        {
            if ( leftInlierMarkers[i] == 1 && rightInlierMarkers[i] == 1)
            {
                inlierMarkers[i] = 1;
            }
        }
        ROS_INFO_STREAM("the number of features before delet point with f: " << leftKpsCurr.size());
        Reduce_Vector(leftKpsRef, inlierMarkers);
        Reduce_Vector(leftKpsCurr, inlierMarkers);
        Reduce_Vector(rightKpsRef, inlierMarkers);
        Reduce_Vector(rightKpsCurr, inlierMarkers);
        Reduce_Vector(trackerID, inlierMarkers);
        Reduce_Vector(trackerCnt, inlierMarkers);
        ROS_INFO_STREAM("the number of features after delet point with f: " << leftKpsCurr.size());
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
    }
    for ( auto& num : trackerCnt)           
        num++;
    
    // 如果需要发布数据，则删除帧间误匹配，增补新的特征点
    if ( pubThisFrame )
    {
        Delet_Point_With_F();
        Set_Mask();
        leftKpsAdd.clear();
        rightKpsAdd.clear();
        int featureMaxCnt = TRACKER_NUM - static_cast<int>(leftKpsCurr.size());
        if ( featureMaxCnt > 0)
        {
            goodFeaturesToTrack(leftImgCurr, leftKpsAdd, featureMaxCnt, 0.01, TRACKER_DIS, mask);

            vector<unsigned char> inlierMarkers(0);
            Stereo_Match(leftKpsAdd, rightKpsAdd, inlierMarkers);

            Reduce_Vector(leftKpsAdd, inlierMarkers);
            Reduce_Vector(rightKpsAdd, inlierMarkers);
        }
        Add_Points();   
    }
    
    for( unsigned int i = 0;; i++ )
    {
        bool completed = false;
        completed |= Update_Tracker_ID(i);
        if (!completed)
            break;
    }

    leftPyramidRef = leftPyramid;
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
    
    Triangulate_Points(leftKpsUndistorted, rightKpsUndistorted);

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

    if (pubMatchImage.getNumSubscribers() > 0)
    {
        Mat outImg(camResolution[1], camResolution[0]*2, CV_8UC3);
        cvtColor(leftImagePtr->image, outImg.colRange(0, camResolution[0]), CV_GRAY2RGB);
        cvtColor(rightImagePtr->image, outImg.colRange(camResolution[0], camResolution[0]*2), CV_GRAY2RGB);
        for( unsigned int i = 0; i < trackerID.size(); i++)
        {
            double cnt = min(1.0, 1.0*trackerCnt[i]/TRACKER_SIZE);
            circle(outImg, leftKpsCurr[i], 3, Scalar(0, 255*(1-cnt), 255*cnt), -1);
            circle(outImg, rightKpsCurr[i]+ Point2f(camResolution[0], 0.0), 3, Scalar(0, 255*(1-cnt), 255*cnt), -1);
            line(outImg, leftKpsCurr[i], rightKpsCurr[i]+ Point2f(camResolution[0], 0.0), Scalar(255, 0, 0), 1, LINE_AA);
        }
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
    uint64 timeBegin = ros::Time::now().toNSec();
    if ( state == FIRST_IMAGE )
    {
        state = NOT_FIRST_IMAGE;        // 状态转换
        currImageTime = firstImageTime = leftImg->header.stamp.toSec();
        return;
    }
    if ( (leftImg->header.stamp.toSec()-currImageTime) > 1.0 )
    {
        ROS_WARN("image fault! reset the feature tracker!");
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

    Create_Image_Pyramid();

    Find_Image_Feature();

    if( pubThisFrame )
    {
        pubCnt++;
        Publish_Info();
    }
    uint64 timeEnd = ros::Time::now().toNSec();
    ROS_INFO_STREAM("code cost time: " << (timeEnd - timeBegin) << " ns");
}

}
