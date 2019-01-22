#include <pose_estimate/pose_estimate.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimate");
    slam_mono::PoseEstimate::ptr a(new slam_mono::PoseEstimate);
    ros::spin();
    return 0;
}