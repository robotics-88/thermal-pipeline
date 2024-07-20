/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/thermal_pipeline_wrapper.h"

#include <opencv2/imgcodecs.hpp>
#include <ros/package.h>

// TODO this is just a hack to make a video quickly, replace using this vector with getting the actual GPS coordinates
std::vector<std::string> latlong = {
    "37.03, -121.746",
    "37.031, -121.745"
};

namespace thermal_pipeline
{
ThermalWrapper::ThermalWrapper(ros::NodeHandle& node)
    : nh_(node)
    , private_nh_("~")
    , thermal_handler_(nh_)
{
    std::string thermal_topic = "/thermal_cam/image_rect_color";
    std::string info_topic = "/thermal_cam/camera_info";
    private_nh_.param<std::string>("image_topic", thermal_topic, thermal_topic);
    private_nh_.param<std::string>("camera_info_topic", info_topic, info_topic);

    thermal_cam_subscriber_.subscribe(nh_, thermal_topic, 10);
    thermal_info_subscriber_.subscribe(nh_, info_topic, 10);

    sync_.reset(new Sync(MySyncPolicy(10), thermal_cam_subscriber_, thermal_info_subscriber_));
    sync_->registerCallback(boost::bind(&ThermalWrapper::thermalImgCallback, this, _1, _2));

    thermal_pub_ = nh_.advertise<sensor_msgs::Image>("/thermal_contours", 10);
    thermal_flagged_pub_ = nh_.advertise<sensor_msgs::Image>("/thermal_flagged", 10);
}

ThermalWrapper::~ThermalWrapper() {
}

void ThermalWrapper::thermalImgCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::CameraInfoConstPtr &img_info) {
    // Convert image to cv Mat
    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGRA8); 
    cv::Mat thermal_mat;
    thermal_mat = cv_ptr->image.clone();

    // Grayscale
    thermal_handler_.convertToGray(thermal_mat);

    // Get contours
    cv::Mat thermal_contour_mat;
    thermal_handler_.thermalContours(thermal_mat, thermal_contour_mat);

    // Get centers
    cv::Mat flagged_mat = cv_ptr->image.clone();
    // cv::cvtColor(flagged_mat, flagged_mat, CV_GRAY2BGR);
    std::vector<cv::Point> centers;
    thermal_handler_.contourCenters(centers);
    addFlagIcon(centers, flagged_mat);

    // Publish contour image
    cv_bridge::CvImage thermal_contour_msg;
    thermal_contour_msg.header   = img->header; // Same timestamp and tf frame as input image
    thermal_contour_msg.encoding = sensor_msgs::image_encodings::BGR8;
    thermal_contour_msg.image    = thermal_contour_mat;
    thermal_pub_.publish(thermal_contour_msg.toImageMsg());

    // Publish flagged image
    thermal_contour_msg.encoding = sensor_msgs::image_encodings::BGRA8;
    thermal_contour_msg.image    = flagged_mat;
    thermal_flagged_pub_.publish(thermal_contour_msg.toImageMsg());
}

// TODO make new subclass for image editing and move both 2 methods below?

void ThermalWrapper::addFlagIcon(const std::vector<cv::Point> positions, cv::Mat &mat) {
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

bool ThermalWrapper::alphaBlend(const cv::Point upper_left, const cv::Mat overlay, cv::Mat &background, int index) {
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