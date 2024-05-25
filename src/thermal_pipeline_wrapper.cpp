/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/thermal_pipeline_wrapper.h"

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
    cv::Mat thermal_contours;
    thermal_handler_.thermalContours(thermal_mat, thermal_contours);

    // Publish contour image
    cv_bridge::CvImage thermal_contour_msg;
    thermal_contour_msg.header   = img->header; // Same timestamp and tf frame as input image
    thermal_contour_msg.encoding = sensor_msgs::image_encodings::BGR8;
    thermal_contour_msg.image    = thermal_contours;
    thermal_pub_.publish(thermal_contour_msg.toImageMsg());
}

}