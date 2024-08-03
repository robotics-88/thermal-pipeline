/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef IMAGE_ANNOTATION_H_
#define IMAGE_ANNOTATION_H_

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core.hpp>

namespace thermal_pipeline {
/**
 * @class ImageAnnotation
 * @brief A class for annotating thermal imagery
 */
class ImageAnnotator{
    public:
        ImageAnnotator(ros::NodeHandle& node);

        ~ImageAnnotator();

        void drawContours(const std::vector<std::vector<cv::Point> > &contours, cv::Mat &img);
        void addFlagIcon(const std::vector<cv::Point> &positions, const std::vector<geometry_msgs::Point> &gps_centers, cv::Mat &mat);
        bool alphaBlend(const cv::Point upper_left, const std::string label, cv::Mat &background, int index);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        cv::Mat gps_icon_mat_;

};
}

#endif