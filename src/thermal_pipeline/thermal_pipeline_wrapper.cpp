/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/thermal_pipeline_wrapper.h"
#include <messages_88/Geopoint.h>

#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace thermal_pipeline
{
ThermalWrapper::ThermalWrapper(ros::NodeHandle& node)
    : nh_(node)
    , private_nh_("~")
    , tf_listener_(tf_buffer_)
    , map_frame_("map")
    , use_rviz_(true)
    , thermal_handler_(nh_)
    , image_annotator_(nh_)
{
    std::string thermal_topic = "/thermal_cam/image_rect_color";
    std::string info_topic = "/thermal_cam/camera_info";
    private_nh_.param<std::string>("image_topic", thermal_topic, thermal_topic);
    private_nh_.param<std::string>("camera_info_topic", info_topic, info_topic);
    private_nh_.param<std::string>("map_frame", map_frame_, map_frame_);
    private_nh_.param<bool>("use_rviz", use_rviz_, use_rviz_);

    geo_client_ = nh_.serviceClient<messages_88::Geopoint>("/slam2geo");

    thermal_cam_subscriber_.subscribe(nh_, thermal_topic, 10);
    thermal_info_subscriber_.subscribe(nh_, info_topic, 10);
    secondary_cam_subscriber_.subscribe(nh_, "/mapir_rgn/image_raw", 10);
    secondary_info_subscriber_.subscribe(nh_, "/mapir_rgn/camera_info", 10);

    sync_.reset(new Sync(MySyncPolicy(10), thermal_cam_subscriber_, thermal_info_subscriber_, secondary_cam_subscriber_, secondary_info_subscriber_));
    sync_->registerCallback(boost::bind(&ThermalWrapper::thermalImgCallback, this, _1, _2, _3, _4));

    thermal_contour_pub_ = nh_.advertise<sensor_msgs::Image>("/thermal_contours", 10);
    second_contour_pub_ = nh_.advertise<sensor_msgs::Image>("/secondary_contours", 10);
    thermal_flagged_pub_ = nh_.advertise<sensor_msgs::Image>("/thermal_flagged", 10);
}

ThermalWrapper::~ThermalWrapper() {
}

void ThermalWrapper::thermalImgCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::CameraInfoConstPtr &img_info, const sensor_msgs::ImageConstPtr &second_img, const sensor_msgs::CameraInfoConstPtr &second_img_info) {
    // Convert image to cv Mat
    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGRA8); 
    cv::Mat thermal_mat;
    thermal_mat = cv_ptr->image.clone();
    cv::Mat flagged_mat = cv_ptr->image.clone();

    cv_ptr = cv_bridge::toCvCopy(second_img, sensor_msgs::image_encodings::BGR8); 
    cv::Mat second_mat;
    second_mat = cv_ptr->image.clone();
    // TODO this is only needed for Mapir RGN, update handling later
    std::vector<cv::Mat> channels(3);
    cv::split(second_mat, channels);
    cv::Mat nir = channels.at(0);

    // Grayscale
    thermal_handler_.convertToGray(thermal_mat);

    // Process primary image
    std::vector<cv::Point3d> projected_centers;
    std::vector<cv::Point> centers;
    cv::Mat contour_mat;
    double min = 200;
    double max = 255;
    processSingleImage(thermal_mat, img_info, min, max, contour_mat, centers, projected_centers);

    // Process secondary image
    std::vector<cv::Point3d> projected_centers2;
    std::vector<cv::Point> ignore;
    cv::Mat contour_mat2;
    min = 190;
    max = 255;
    processSingleImage(nir, second_img_info, min, max, contour_mat2, ignore, projected_centers2);

    // Transform centers
    std::vector<geometry_msgs::Point> gps_centers;
    bool did_transform = getImagePointsInGPS(projected_centers, img->header, gps_centers);
    if (!did_transform) {
        return;
    }

    // Label images
    image_annotator_.addFlagIcon(centers, gps_centers, flagged_mat);

    if (use_rviz_) {
        // Publish flagged image
        cv_bridge::CvImage thermal_contour_msg;
        thermal_contour_msg.header   = img->header; // Same timestamp and tf frame as input image
        thermal_contour_msg.encoding = sensor_msgs::image_encodings::BGRA8;
        thermal_contour_msg.image    = flagged_mat;
        thermal_flagged_pub_.publish(thermal_contour_msg.toImageMsg());

        thermal_contour_msg.header   = img->header; // Same timestamp and tf frame as input image
        thermal_contour_msg.encoding = sensor_msgs::image_encodings::BGR8;
        thermal_contour_msg.image    = contour_mat;
        thermal_contour_pub_.publish(thermal_contour_msg.toImageMsg());

        thermal_contour_msg.header   = second_img->header; // Same timestamp and tf frame as input image
        thermal_contour_msg.encoding = sensor_msgs::image_encodings::BGR8;
        thermal_contour_msg.image    = contour_mat2;
        second_contour_pub_.publish(thermal_contour_msg.toImageMsg());
    }

}

void ThermalWrapper::processSingleImage(const cv::Mat &image, const sensor_msgs::CameraInfoConstPtr &img_info, double min, double max, cv::Mat &contour_image, std::vector<cv::Point> &centers, std::vector<cv::Point3d> &contour_centers) {
    // Get contours
    thermal_handler_.thermalContours(image, contour_image, min, max);

    // Get projected centers
    thermal_handler_.contourCenters(*img_info, centers, contour_centers);
}

bool ThermalWrapper::getImagePointsInGPS(const std::vector<cv::Point3d> &centers, const std_msgs::Header &header, std::vector<geometry_msgs::Point> &gps_centers) {
    if (!geo_client_.exists()) {
        return false;
    }
    geometry_msgs::TransformStamped transform_image2map;
    std::string transform_error;
    try{
        transform_image2map = tf_buffer_.lookupTransform(map_frame_, header.frame_id, header.stamp);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return false;
    }
    for (int i = 0; i < centers.size(); i++) {
        // Project to map
        geometry_msgs::PointStamped image_point_3d, map_point;
        image_point_3d.header = header;
        image_point_3d.point.x = centers.at(i).x;
        image_point_3d.point.y = centers.at(i).y;
        image_point_3d.point.z = centers.at(i).z;
        tf2::doTransform(image_point_3d, map_point, transform_image2map);

        // Get lat/long
        geometry_msgs::Point geo_map_point;
        geo_map_point.x = map_point.point.x;
        geo_map_point.y = map_point.point.y;
        geo_map_point.z = map_point.point.z;
        messages_88::Geopoint geo_request;
        geo_request.request.slam_position = geo_map_point;
        geo_client_.call(geo_request);
        geometry_msgs::Point world_point;
        world_point.x = geo_request.response.latitude;
        world_point.y = geo_request.response.longitude;

        gps_centers.push_back(world_point);
    }
    return true;
}

}