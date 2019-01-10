#ifndef _FEATURE_TRACKER_H_
#define _FEATURE_TRACKER_H_

#include <feature_tracker/my_include.h>
#include <feature_tracker/config.h>

namespace enc = sensor_msgs::image_encodings;

namespace slam_mono
{
    class FeatureTracker
    {    
    private:
        typedef long long int LONGTYPE;
        struct Feature_Data
        {
            LONGTYPE id;
            LONGTYPE lifeTime;
            Point2f leftPoints;
            Point2f rightPoints;
        };
        
        enum FeatureState
        {
            FIRST_IMAGE = -1,
            NOT_FIRST_IMAGE = 0
        };
        FeatureState state;
        
        // 相关宏定义及参数
        double firstImageTime;
        double currImageTime;
        double F_THRESHOLD;
        bool pubThisFrame;
        bool EQUALIZE;
        bool SHOW_TRACKER;
        int pubCnt;
        int FREQ;
        int WIN_SIZE;
        int TRACKER_SIZE;
        int BOARD_SIZE;
        int TRACKER_DIS;
        int TRACKER_NUM;
        int PYRAMID_LEVEL;
        

        // 相机相关参数
        string distortionModel;
        string LEFT_TOPICS;
        string RIGHT_TOPICS;
        Vec2i camResolution;
        Vec4d leftIntrinsics;
        Vec4d rightIntrinsics;
        Vec4d leftDistortionCoeffs;
        Vec4d rightDistortionCoeffs;
        Mat T_left2right;
        Matx33d r_left2right;
        Vec3d t_left2right;

        // ROS相关初始化
        ros::NodeHandle n;
        ros::Publisher pubLeftMatchImage;
        ros::Publisher pubRightMatchImage;
        ros::Subscriber imgSub;
        message_filters::Subscriber<sensor_msgs::Image> leftSub;
        message_filters::Subscriber<sensor_msgs::Image> rightSub;
        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> stereoSub;

        // 图像获取与图像金字塔
        cv_bridge::CvImageConstPtr leftImagePtr;
        cv_bridge::CvImageConstPtr rightImagePtr;
        vector<Mat> leftPyramid;
        vector<Mat> leftPyramidRef;
        vector<Mat> rightPyramid;

        // 特征提取相关定义
        LONGTYPE n_id;
        Mat mask;
        vector<LONGTYPE> trackerID;
        vector<LONGTYPE> trackerCnt;
        vector<Point2f> leftKpsCurr;
        vector<Point2f> leftKpsRef;
        vector<Point2f> rightKpsCurr;
        vector<Point2f> rightKpsRef;
        vector<Point2f> leftKpsAdd;
        vector<Point2f> rightKpsAdd;

        // 相关函数
        bool Load_Parameters(void);
        bool Create_RosIO(void);
        void Create_Image_Pyramid(void);
        void Find_Image_Feature(void);
        bool Point_In_Border(const Point2f& pt);
        void Stereo_Match(const vector<Point2f>& leftPoints, vector<Point2f>& rightPoints, vector<unsigned char>& inlierMarkers);
        void Undistorted_Points(const vector<Point2f> ptsIn, vector<Point2f>& ptsOut, 
                               const Vec4d& intrinsics, const Vec4d& distortionCoeffs, 
                               const Matx33d& rotation = Matx33d::eye(), const Vec4d& newIntrinsics = Vec4d(1, 1, 0, 0));

        void Distortion_Points(const vector<Point2f> ptsIn, vector<Point2f>& ptsOut, 
                                            const Vec4d& intrinsics, const Vec4d& distortionCoeffs);
        void Add_Points(void);
        bool Update_Tracker_ID(int i);
        void Set_Mask(void);
        void Tracker_Feature(void);
        void Delet_Point_With_F(void);

        template <typename T>
        void Reduce_Vector(vector<T>& v, vector<unsigned char> status)
        {
            if ( v.size() != status.size() )
            {
                ROS_WARN("the input size of vector(%lu) and markers(%lu) does not match: ", v.size(), status.size());
            }
            int num = 0;
            for (int i = 0; i < int(v.size()); i++ )
            {
                if (status[i])
                {
                    v[num++] = v[i];
                }
            }
            v.resize(num);
        }
    public:
        typedef shared_ptr<FeatureTracker> ptr;

        FeatureTracker(void);
         ~FeatureTracker(void);
         
        void Stereo_Callback(const sensor_msgs::ImageConstPtr& leftImg, const sensor_msgs::ImageConstPtr& rightImg);

    };
}


#endif