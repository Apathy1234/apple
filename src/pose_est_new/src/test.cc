#include <pose_est_new/pose_est_new.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_est_new");
    slam_mono::PoseEst::ptr a(new slam_mono::PoseEst);
    ros::spin();
    return 0;
}