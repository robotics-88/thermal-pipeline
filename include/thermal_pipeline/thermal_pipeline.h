/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef THERMAL_PIPELINE_H_
#define THERMAL_PIPELINE_H_

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

namespace thermal_pipeline {
/**
 * @class Thermal
 * @brief A class for processing thermal imagery
 */
class Thermal{
    public:
        Thermal(ros::NodeHandle& node);

        ~Thermal();

        void convertToGray(cv::Mat &img);
        int thermalContours(const cv::Mat &img, const double min, const image_geometry::PinholeCameraModel model, std::vector<std::vector<cv::Point> > &contours);
        void contourCenters(const image_geometry::PinholeCameraModel model, std::vector<cv::Point> &centers, std::vector<cv::Point3d> &projected_centers);
        void projectContour(const image_geometry::PinholeCameraModel model, const std::vector<cv::Point> &contour, std::vector<cv::Point3d> &projected_contour);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        std::vector<std::vector<cv::Point> > contours_;

};
}

#endif