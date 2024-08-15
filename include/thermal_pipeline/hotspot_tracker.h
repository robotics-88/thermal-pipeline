/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef HOTSPOT_TRACKER_H_
#define HOTSPOT_TRACKER_H_

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>


namespace thermal_pipeline {
/**
 * @class HotspotTracker
 * @brief A class for filtering and storing hotspot detections
 */
class HotspotTracker{
    public:
        HotspotTracker(ros::NodeHandle& node);

        ~HotspotTracker();

        void nirFilter(const std::vector<std::vector<geometry_msgs::PointStamped> > &thermal_contours, const std::vector<std::vector<geometry_msgs::PointStamped> > &second_contours, std::vector<int> &indices);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        struct Hotspot {
            geometry_msgs::Polygon contour;
            ros::Time last_detection_;
            int num_detections_;
        };

        std::vector<Hotspot> hotspots_;

};
}

#endif