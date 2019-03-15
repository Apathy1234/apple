#ifndef _MY_INCLUDE_H_
#define _MY_INCLUDE_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <feature_tracker/CameraTrackerResult.h>
#include <feature_tracker/FeatureTrackerResult.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <memory>
#include <cmath>

using namespace std;
using namespace cv;

typedef unsigned short ushort;

#endif 