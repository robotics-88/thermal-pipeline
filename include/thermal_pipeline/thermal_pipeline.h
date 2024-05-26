/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef THERMAL_PIPELINE_H_
#define THERMAL_PIPELINE_H_

#include <string>

#include <ros/ros.h>

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

        int thermalContours(const cv::Mat &img, cv::Mat &img_contours);
        void convertToGray(cv::Mat &img);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

};
}

#endif