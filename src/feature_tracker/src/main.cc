#include <feature_tracker/my_include.h>
#include <feature_tracker/feature_tracker.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n;

    slam_mono::FeatureTracker::ptr featureTracker(new slam_mono::FeatureTracker);
    
    ros::spin();
    return 0;
}