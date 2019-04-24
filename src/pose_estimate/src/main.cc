#include <pose_estimate/loosely_vio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimate");
    slam_mono::Loosely_vio::ptr a(new slam_mono::Loosely_vio);
    ros::spin();
    return 0;
}