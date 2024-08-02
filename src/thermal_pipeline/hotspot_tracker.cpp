/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/hotspot_tracker.h"

#include <ros/package.h>

#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>

namespace thermal_pipeline
{
HotspotTracker::HotspotTracker(ros::NodeHandle& node)
    : nh_(node)
    , private_nh_("~")
{
}

HotspotTracker::~HotspotTracker() {
}

void HotspotTracker::nirFilter(const std::vector<std::vector<geometry_msgs::PointStamped> > &thermal_contours, const std::vector<std::vector<geometry_msgs::PointStamped> > &second_contours) {
    // Remove any thermal contour without a corresponding NIR contour
    for (int ii = 0; ii < thermal_contours.size(); ii++) {
        // Find overlapping
    }
}

}