/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef THERMAL_PIPELINE_WRAPPER_H_
#define THERMAL_PIPELINE_WRAPPER_H_

#include <string>

#include <ros/ros.h>

#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>

#include "thermal_pipeline/thermal_pipeline.h"
#include "thermal_pipeline/image_annotation.h"
#include "thermal_pipeline/hotspot_tracker.h"

namespace thermal_pipeline {
/**
 * @class ThermalWrapper
 * @brief A class for pub/sub and wrapper for thermal imagery
 */
class ThermalWrapper {
    public:
        ThermalWrapper(ros::NodeHandle& node);

        ~ThermalWrapper();


    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::string map_frame_;
        bool use_rviz_;

        ros::ServiceClient geo_client_;

        thermal_pipeline::Thermal thermal_handler_;
        thermal_pipeline::ImageAnnotator image_annotator_;
        thermal_pipeline::HotspotTracker hotspot_tracker_;

        ros::Publisher thermal_contour_pub_;
        ros::Publisher second_contour_pub_;
        ros::Publisher filtered_contour_pub_;
        ros::Publisher thermal_flagged_pub_;
        message_filters::Subscriber<sensor_msgs::Image> thermal_cam_subscriber_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> thermal_info_subscriber_;
        message_filters::Subscriber<sensor_msgs::Image> secondary_cam_subscriber_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> secondary_info_subscriber_;

        bool camera_model_set_;
        image_geometry::PinholeCameraModel thermal_model_;
        image_geometry::PinholeCameraModel second_model_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;

        void thermalImgCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::CameraInfoConstPtr &img_info, const sensor_msgs::ImageConstPtr &second_img, const sensor_msgs::CameraInfoConstPtr &second_img_info);
        bool transformContours(const image_geometry::PinholeCameraModel model, const std_msgs::Header header, const std::vector<std::vector<cv::Point> > &contours, std::vector<std::vector<geometry_msgs::PointStamped> > &map_contours);
        void mapContoursToImage();
        void processSingleImage(const cv::Mat &image, const sensor_msgs::CameraInfoConstPtr &img_info, double min, double max, cv::Mat &contour_image, std::vector<cv::Point> &centers, std::vector<cv::Point3d> &contour_centers);
        bool getImagePointsInGPS(const std::vector<cv::Point3d> &centers, const std_msgs::Header &header, std::vector<geometry_msgs::Point> &gps_centers);
        bool transformCVPoint(const cv::Point3d point, const geometry_msgs::TransformStamped transform_image2map, std_msgs::Header header, geometry_msgs::PointStamped &map_point);
};

}

#endif