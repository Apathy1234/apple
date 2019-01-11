#ifndef _FEATURE_TRACKER_NODELET_H_
#define _FEATURE_TRACKER_NODELET_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <feature_tracker/feature_tracker.h>

namespace slam_mono
{
    class FeatureTrackerNodelet : public nodelet::Nodelet
    {
    public:
        FeatureTrackerNodelet(void);
        ~FeatureTrackerNodelet(void);
    private:
        virtual void onInit();
        FeatureTracker::ptr featureTracker;
    };
}


#endif 