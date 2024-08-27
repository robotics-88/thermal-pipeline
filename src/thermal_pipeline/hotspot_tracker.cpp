/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/hotspot_tracker.h"

#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> BgPoint;
typedef bg::model::polygon<BgPoint> BgPolygon;

namespace thermal_pipeline
{
HotspotTracker::HotspotTracker()
{
}

HotspotTracker::~HotspotTracker() {
}

void HotspotTracker::nirFilter(const std::vector<std::vector<geometry_msgs::msg::PointStamped> > &thermal_contours, const std::vector<std::vector<geometry_msgs::msg::PointStamped> > &second_contours, std::vector<int> &indices) {
    std::vector<std::vector<geometry_msgs::msg::PointStamped> > filtered_contours;
    // Remove any thermal contour without a corresponding NIR contour
    for (int ii = 0; ii < thermal_contours.size(); ii++) {
        std::vector<geometry_msgs::msg::PointStamped> polygon_map = thermal_contours.at(ii);
        BgPolygon bg_poly;
        for (const auto &point : polygon_map) {
            BgPoint bgpoint(point.point.x, point.point.y);
            bg_poly.outer().push_back(bgpoint);
        }
        bg_poly.outer().push_back(bg_poly.outer().front());
        // double a = bg::area(bg_poly);
        // Find overlapping
        for (int jj = 0; jj < second_contours.size(); jj++) {
            std::vector<geometry_msgs::msg::PointStamped> polygon2 = second_contours.at(jj);
            BgPolygon bg_poly2;
            for (const auto &point2 : polygon2) {
                BgPoint bgpoint(point2.point.x, point2.point.y);
                bg_poly2.outer().push_back(bgpoint);
            }
            bg_poly2.outer().push_back(bg_poly2.outer().front());
            std::deque<BgPolygon> output;
            if (bg::intersection(bg_poly, bg_poly2, output)) {
                filtered_contours.push_back(thermal_contours.at(ii));
                indices.push_back(ii);
                break;
            }
        }
    }
}

}