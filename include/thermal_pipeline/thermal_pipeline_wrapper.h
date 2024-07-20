/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef THERMAL_PIPELINE_WRAPPER_H_
#define THERMAL_PIPELINE_WRAPPER_H_

#include <string>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "thermal_pipeline/thermal_pipeline.h"
#include "thermal_pipeline/image_annotation.h"

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

        thermal_pipeline::Thermal thermal_handler_;
        thermal_pipeline::ImageAnnotator image_annotator_;

        ros::Publisher thermal_pub_;
        ros::Publisher thermal_flagged_pub_;
        message_filters::Subscriber<sensor_msgs::Image> thermal_cam_subscriber_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> thermal_info_subscriber_;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;

        void thermalImgCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::CameraInfoConstPtr &img_info);
};

}

#endif