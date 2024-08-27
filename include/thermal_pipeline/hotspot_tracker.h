/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef HOTSPOT_TRACKER_H_
#define HOTSPOT_TRACKER_H_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>


namespace thermal_pipeline {
/**
 * @class HotspotTracker
 * @brief A class for filtering and storing hotspot detections
 */
class HotspotTracker {
    public:
        HotspotTracker();
        ~HotspotTracker();

        void nirFilter(const std::vector<std::vector<geometry_msgs::msg::PointStamped> > &thermal_contours, const std::vector<std::vector<geometry_msgs::msg::PointStamped> > &second_contours, std::vector<int> &indices);

    private:
        struct Hotspot {
            geometry_msgs::msg::Polygon contour;
            rclcpp::Time last_detection_;
            int num_detections_;
        };

        std::vector<Hotspot> hotspots_;

};
}

#endif