#include <feature_tracker/my_include.h>
#include <feature_tracker/feature_tracker_nodelet.h>

PLUGINLIB_EXPORT_CLASS(slam_mono::FeatureTrackerNodelet, nodelet::Nodelet);

namespace slam_mono
{
    
    FeatureTrackerNodelet::FeatureTrackerNodelet(void)
    {

    }

    FeatureTrackerNodelet::~FeatureTrackerNodelet(void)
    {

    }

    void FeatureTrackerNodelet::onInit()
    {
        ROS_INFO("Start feature tracker...");
        featureTracker.reset(new FeatureTracker(getNodeHandle()));
    }
}

