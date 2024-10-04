/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/thermal_pipeline_wrapper.h"

#include <geometry_msgs/msg/point_stamped.hpp>
using namespace std::chrono_literals;

namespace thermal_pipeline
{
ThermalWrapper::ThermalWrapper(const rclcpp::NodeOptions& options)
    : Node("thermal_wrapper", options)
    , map_frame_("map")
    , use_rviz_(true)
    , thermal_handler_()
    , image_annotator_()
    , hotspot_tracker_()
{
}

ThermalWrapper::~ThermalWrapper() {
}

void ThermalWrapper::initialize() {
    std::string thermal_topic = "/thermal_cam/image_rect_color";
    std::string info_topic = "/thermal_cam/camera_info";
    this->declare_parameter("image_topic", thermal_topic);
    this->declare_parameter("camera_info_topic", info_topic);
    this->declare_parameter("map_frame", map_frame_);
    this->declare_parameter("use_rviz", use_rviz_);
    this->get_parameter("image_topic", thermal_topic);
    this->get_parameter("camera_info_topic", info_topic);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("use_rviz", use_rviz_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    geo_client_ = this->create_client<messages_88::srv::Geopoint>("/slam2geo");
    
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    thermal_cam_subscriber_.subscribe(this, thermal_topic, rmw_qos_profile);
    thermal_info_subscriber_.subscribe(this, info_topic, rmw_qos_profile);
    secondary_cam_subscriber_.subscribe(this, "/mapir_rgb/image_raw", rmw_qos_profile);
    secondary_info_subscriber_.subscribe(this, "/mapir_rgb/camera_info", rmw_qos_profile);

    sync_.reset(new message_filters::Synchronizer<approximate_policy>(approximate_policy(30), thermal_cam_subscriber_, thermal_info_subscriber_, secondary_cam_subscriber_, secondary_info_subscriber_));
    sync_->registerCallback(std::bind(&ThermalWrapper::thermalImgCallback,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2,
                                             std::placeholders::_3,
                                             std::placeholders::_4));

    thermal_contour_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/thermal_contours", 10);
    second_contour_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/secondary_contours", 10);
    filtered_contour_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/filtered_contours", 10);
    thermal_flagged_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/thermal_flagged", 10);
}

void ThermalWrapper::thermalImgCallback(const sensor_msgs::msg::Image::ConstSharedPtr img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr img_info, const sensor_msgs::msg::Image::ConstSharedPtr second_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr second_img_info) {
    if (!camera_model_set_) {
        thermal_model_.fromCameraInfo(img_info);
        second_model_.fromCameraInfo(second_img_info);
        camera_model_set_ = true;
    }
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

    // Save BGRA mats for contour drawing
    cv::Mat contour_mat = thermal_mat.clone();
    cv::Mat contour_mat2 = nir.clone();
    cv::cvtColor(contour_mat2, contour_mat2, cv::COLOR_GRAY2BGR);
    cv::Mat contour_mat_filtered = thermal_mat.clone();

    // Grayscale
    thermal_handler_.convertToGray(thermal_mat);

    // Process primary image
    std::vector<cv::Point3d> projected_centers;
    std::vector<cv::Point> centers;
    std::vector<std::vector<cv::Point> > thermal_contours;
    std::vector<std::vector<geometry_msgs::msg::PointStamped> > thermal_contours_map;
    double min = 200;
    thermal_handler_.thermalContours(thermal_mat, min, thermal_model_, thermal_contours);
    bool got_tf = transformContours(thermal_model_, img->header, thermal_contours, thermal_contours_map);
    if (!got_tf) {
        RCLCPP_INFO(this->get_logger(), "Thermal pipeline stopped, no TF for contours");
        return;
    }
    image_annotator_.drawContours(thermal_contours, contour_mat);

    // Process secondary image
    std::vector<cv::Point3d> projected_centers2;
    std::vector<cv::Point> ignore;
    std::vector<std::vector<cv::Point> > tmp_contours;
    std::vector<std::vector<geometry_msgs::msg::PointStamped> > second_contours_map;
    min = 190;
    thermal_handler_.thermalContours(nir, min, second_model_, tmp_contours);
    transformContours(second_model_, second_img->header, tmp_contours, second_contours_map);
    image_annotator_.drawContours(tmp_contours, contour_mat2);

    std::vector<int> indices;
    hotspot_tracker_.nirFilter(thermal_contours_map, second_contours_map, indices);
    std::vector<std::vector<cv::Point> > filtered_cv_contours;
    for (int ii = 0; ii < indices.size(); ii++) {
        filtered_cv_contours.push_back(thermal_contours.at(indices.at(ii)));
    }
    image_annotator_.drawContours(filtered_cv_contours, contour_mat_filtered);

    // Transform centers
    std::vector<geometry_msgs::msg::Point> gps_centers;
    bool did_transform = getImagePointsInGPS(projected_centers, img->header, gps_centers);
    if (!did_transform) {
        RCLCPP_INFO(this->get_logger(), "Thermal pipeline stopped, no TF for GPS flags");
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
        thermal_flagged_pub_->publish(*(thermal_contour_msg.toImageMsg()).get());

        thermal_contour_msg.header   = img->header; // Same timestamp and tf frame as input image
        thermal_contour_msg.encoding = sensor_msgs::image_encodings::BGRA8;
        thermal_contour_msg.image    = contour_mat;
        thermal_contour_pub_->publish(*(thermal_contour_msg.toImageMsg()).get());

        thermal_contour_msg.header   = second_img->header; // Same timestamp and tf frame as input image
        thermal_contour_msg.encoding = sensor_msgs::image_encodings::BGR8;
        thermal_contour_msg.image    = contour_mat2;
        second_contour_pub_->publish(*(thermal_contour_msg.toImageMsg()).get());

        thermal_contour_msg.header   = img->header; // Same timestamp and tf frame as input image
        thermal_contour_msg.encoding = sensor_msgs::image_encodings::BGRA8;
        thermal_contour_msg.image    = contour_mat_filtered;
        filtered_contour_pub_->publish(*(thermal_contour_msg.toImageMsg()).get());
    }

}

bool ThermalWrapper::transformContours(const image_geometry::PinholeCameraModel model, const std_msgs::msg::Header header, const std::vector<std::vector<cv::Point> > &contours, std::vector<std::vector<geometry_msgs::msg::PointStamped> > &map_contours) {
    geometry_msgs::msg::TransformStamped transform_thermal2map;
    std::string transform_error;
    try{
        transform_thermal2map = tf_buffer_->lookupTransform(map_frame_, header.frame_id, header.stamp);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Transform for contours failed with %s", ex.what());
        return false;
    }
    for (int ii = 0; ii < contours.size(); ii++) {
        std::vector<cv::Point3d> contour_3d;
        thermal_handler_.projectContour(model, contours.at(ii), contour_3d);
        std::vector<geometry_msgs::msg::PointStamped> contour_map;
        for (int jj = 0; jj < contour_3d.size(); jj++) {
            geometry_msgs::msg::PointStamped map_point;
            transformCVPoint(contour_3d.at(jj), transform_thermal2map, header, map_point);
            contour_map.push_back(map_point);
        }
        map_contours.push_back(contour_map);
    }
    return true;
}

bool ThermalWrapper::transformCVPoint(const cv::Point3d point, const geometry_msgs::msg::TransformStamped transform_image2map, const std_msgs::msg::Header header, geometry_msgs::msg::PointStamped &map_point) {
    geometry_msgs::msg::PointStamped image_point_3d;
    image_point_3d.header = header;
    image_point_3d.point.x = point.x;
    image_point_3d.point.y = point.y;
    image_point_3d.point.z = point.z;
    tf2::doTransform(image_point_3d, map_point, transform_image2map);
}

bool ThermalWrapper::getImagePointsInGPS(const std::vector<cv::Point3d> &centers, const std_msgs::msg::Header &header, std::vector<geometry_msgs::msg::Point> &gps_centers) {
    if (!geo_client_->wait_for_service(5s)) {
        return false;
    }
    geometry_msgs::msg::TransformStamped transform_image2map;
    std::string transform_error;
    try{
        transform_image2map = tf_buffer_->lookupTransform(map_frame_, header.frame_id, header.stamp);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Transform for image points failed with %s", ex.what());
        return false;
    }
    for (int i = 0; i < centers.size(); i++) {
        // Project to map
        geometry_msgs::msg::PointStamped map_point;
        transformCVPoint(centers.at(i), transform_image2map, header, map_point);

        // Get lat/long
        // messages_88::srv::Geopoint geo_request;
        // geo_request.request.slam_position = geo_map_point;
        // geo_client_->call(geo_request);
        auto geo_req = std::make_shared<messages_88::srv::Geopoint::Request>(); 
        geometry_msgs::msg::Point geo_map_point;
        geo_map_point.x = map_point.point.x;
        geo_map_point.y = map_point.point.y;
        geo_map_point.z = map_point.point.z;
        geo_req->slam_position = geo_map_point;
        auto geo_resp = geo_client_->async_send_request(geo_req);
        const std::shared_ptr<rclcpp::Node> geo_srv_node = rclcpp::Node::make_shared("geo_srv_client");
        if (rclcpp::spin_until_future_complete(geo_srv_node, geo_resp) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "General stream rate set");
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service /mavros/set_stream_rate");
        }
        geometry_msgs::msg::Point world_point;
        world_point.x = geo_resp.get()->latitude;
        world_point.y = geo_resp.get()->longitude;

        gps_centers.push_back(world_point);
    }
    return true;
}

}