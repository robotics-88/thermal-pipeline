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
        int thermalContours(const cv::Mat &img, cv::Mat &img_contours);
        void contourCenters(const sensor_msgs::CameraInfo &info, std::vector<cv::Point> &centers, std::vector<cv::Point3d> &projected_centers);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        std::vector<std::vector<cv::Point> > contours_;
        bool camera_model_set_;
        image_geometry::PinholeCameraModel camera_model_;

};
}

#endif