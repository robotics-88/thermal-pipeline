/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef THERMAL_PIPELINE_WRAPPER_H_
#define THERMAL_PIPELINE_WRAPPER_H_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "messages_88/srv/geopoint.hpp"
#include "thermal_pipeline/thermal_pipeline.h"
#include "thermal_pipeline/image_annotation.h"
#include "thermal_pipeline/hotspot_tracker.h"

namespace thermal_pipeline {
/**
 * @class ThermalWrapper
 * @brief A class for pub/sub and wrapper for thermal imagery
 */
class ThermalWrapper : public rclcpp::Node {
    public:
        explicit ThermalWrapper(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        ~ThermalWrapper();
        void initialize();

    private:
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string map_frame_;
        bool use_rviz_;

        rclcpp::Client<messages_88::srv::Geopoint>::SharedPtr geo_client_;

        thermal_pipeline::Thermal thermal_handler_;
        thermal_pipeline::ImageAnnotator image_annotator_;
        thermal_pipeline::HotspotTracker hotspot_tracker_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           thermal_contour_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           second_contour_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           filtered_contour_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           thermal_flagged_pub_;
        message_filters::Subscriber<sensor_msgs::msg::Image>            thermal_cam_subscriber_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo>       thermal_info_subscriber_;
        message_filters::Subscriber<sensor_msgs::msg::Image>            secondary_cam_subscriber_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo>       secondary_info_subscriber_;

        bool camera_model_set_;
        image_geometry::PinholeCameraModel thermal_model_;
        image_geometry::PinholeCameraModel second_model_;

        // std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Temperature, sensor_msgs::msg::Temperature>> temp_sync_;
        // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MySyncPolicy;
        // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        // boost::shared_ptr<Sync> sync_;
        // typedef message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> Sync;
        // Sync sync_;
        using approximate_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
        typedef message_filters::Synchronizer<approximate_policy> Synchronizer;
        std::unique_ptr<Synchronizer> sync_;

        void thermalImgCallback(const sensor_msgs::msg::Image::ConstSharedPtr img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr img_info, const sensor_msgs::msg::Image::ConstSharedPtr second_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr second_img_info);
        bool transformContours(const image_geometry::PinholeCameraModel model, const std_msgs::msg::Header header, const std::vector<std::vector<cv::Point> > &contours, std::vector<std::vector<geometry_msgs::msg::PointStamped> > &map_contours);
        // void mapContoursToImage();
        // void processSingleImage(const cv::Mat &image, const sensor_msgs::msg::CameraInfoConstPtr &img_info, double min, double max, cv::Mat &contour_image, std::vector<cv::Point> &centers, std::vector<cv::Point3d> &contour_centers);
        bool getImagePointsInGPS(const std::vector<cv::Point3d> &centers, const std_msgs::msg::Header &header, std::vector<geometry_msgs::msg::Point> &gps_centers);
        bool transformCVPoint(const cv::Point3d point, const geometry_msgs::msg::TransformStamped transform_image2map, std_msgs::msg::Header header, geometry_msgs::msg::PointStamped &map_point);
};

}

#endif