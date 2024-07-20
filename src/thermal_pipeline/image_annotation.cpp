/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/image_annotation.h"

#include <ros/package.h>

#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>

// TODO this is just a hack to make a video quickly, replace using this vector with getting the actual GPS coordinates
std::vector<std::string> latlong = {
    "37.03, -121.746",
    "37.031, -121.745"
};

namespace thermal_pipeline
{
ImageAnnotator::ImageAnnotator(ros::NodeHandle& node)
    : nh_(node)
    , private_nh_("~")
{
}

ImageAnnotator::~ImageAnnotator() {
}

void ImageAnnotator::addFlagIcon(const std::vector<cv::Point> positions, cv::Mat &mat) {
    cv::Mat smallImage = cv::imread(ros::package::getPath("thermal_88") + "/images/geo-arrow.png", cv::IMREAD_UNCHANGED);
    int img_height = smallImage.rows;
    int img_width = smallImage.cols;
    cv::Mat foreground = cv::Mat::zeros(mat.size(), CV_8UC4);
    int flag_count = 0;
    int flag_max = 2;
    for (int ii = 0; ii < positions.size(); ii++) {
        cv::Point p(positions.at(ii).x - (img_width / 2), positions.at(ii).y - img_height);
        // cv::circle(mat, positions.at(ii), 5, cv::Scalar(177,193,81), -1);
        if (flag_count < flag_max) {
            bool flagged = alphaBlend(p, smallImage, mat, flag_count);
            if (flagged) flag_count++;
        }
    }
}

bool ImageAnnotator::alphaBlend(const cv::Point upper_left, const cv::Mat overlay, cv::Mat &background, int index) {
    // Check if overlay position would fit on background
    cv::Mat destRoi;
    try {
        destRoi = background(cv::Rect(upper_left.x, upper_left.y, overlay.cols, overlay.rows));
    }  catch (...) {
        // TODO truncate or move the tag?
        return false;
    }
    int alpha = 255;
    int white_threshold = 100; // Can make this really low because the flag itself has a red value of 2
    for (int r = 0; r < overlay.rows; r++) {
        for (int c = 0; c < overlay.cols; c++) {
            cv::Point background_point(upper_left.x + c, upper_left.y + r);
            cv::Point overlay_point(c, r);
            cv::Vec4b p = overlay.at<cv::Vec4b>(overlay_point);
            if (p[0] >= white_threshold && p[1] >= white_threshold && p[2] >= white_threshold) {
                // White points should be transparent so leave background as is
                continue;
            }
            else {
                background.at<cv::Vec4b>(background_point) = p;
            }
        }
    }
    cv::putText(background, latlong[index], upper_left, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
}

}