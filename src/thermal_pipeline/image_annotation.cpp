/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include "thermal_pipeline/image_annotation.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>

namespace thermal_pipeline {
ImageAnnotator::ImageAnnotator() {
    gps_icon_mat_ = cv::imread(ament_index_cpp::get_package_share_directory("thermal_88") +
                                   "/config/geo-arrow.png",
                               cv::IMREAD_UNCHANGED);
}

ImageAnnotator::~ImageAnnotator() {}

void ImageAnnotator::drawContours(const std::vector<std::vector<cv::Point>> &contours,
                                  cv::Mat &img) {
    cv::Scalar color = cv::Scalar(0, 0, 255);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(img, contours, (int)i, color, 2, cv::LINE_8);
    }
}

void ImageAnnotator::addFlagIcon(const std::vector<cv::Point> &positions,
                                 const std::vector<geometry_msgs::msg::Point> &gps_centers,
                                 cv::Mat &mat) {
    cv::Mat foreground = cv::Mat::zeros(mat.size(), CV_8UC4);
    int flag_count = 0;
    int flag_max = 2;
    for (int ii = 0; ii < positions.size(); ii++) {
        cv::Point p(positions.at(ii).x - (gps_icon_mat_.cols / 2),
                    positions.at(ii).y - gps_icon_mat_.rows);
        std::string gps_label =
            std::to_string(gps_centers.at(ii).x) + ", " + std::to_string(gps_centers.at(ii).y);
        if (flag_count < flag_max) {
            bool flagged = alphaBlend(p, gps_label, mat, flag_count);
            if (flagged)
                flag_count++;
        }
    }
}

bool ImageAnnotator::alphaBlend(const cv::Point upper_left, const std::string label,
                                cv::Mat &background, int index) {
    // Check if overlay position would fit on background
    cv::Mat destRoi;
    try {
        destRoi = background(
            cv::Rect(upper_left.x, upper_left.y, gps_icon_mat_.cols, gps_icon_mat_.rows));
    } catch (...) {
        // TODO truncate or move the tag?
        return false;
    }
    int alpha = 255;
    int white_threshold =
        100; // Can make this really low because the flag itself has a red value of 2
    for (int r = 0; r < gps_icon_mat_.rows; r++) {
        for (int c = 0; c < gps_icon_mat_.cols; c++) {
            cv::Point background_point(upper_left.x + c, upper_left.y + r);
            cv::Point overlay_point(c, r);
            cv::Vec4b p = gps_icon_mat_.at<cv::Vec4b>(overlay_point);
            if (p[0] >= white_threshold && p[1] >= white_threshold && p[2] >= white_threshold) {
                // White points should be transparent so leave background as is
                continue;
            } else {
                background.at<cv::Vec4b>(background_point) = p;
            }
        }
    }
    cv::putText(background, label, upper_left, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
}

} // namespace thermal_pipeline