/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef THERMAL_CALIBRATOR_H_
#define THERMAL_CALIBRATOR_H_

#include <string>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace thermal_pipeline {
/**
 * @class Calibrator
 * @brief A class for converting images with bounding boxes on detected species into maps
 */
class Calibrator {
    public:
        Calibrator(ros::NodeHandle& node);

        ~Calibrator();

        void imageCallback(const sensor_msgs::Image::ConstPtr &image);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber image_sub_;
        bool first_image_ = true;

        ros::Publisher image_chessboard_pub_;
        ros::Publisher image_rect_pub_;
        ros::Publisher info_pub_;

        sensor_msgs::CameraInfo camera_info_;

        // Calibration objects

        // Creating vector to store vectors of 3D points for each checkerboard image
        std::vector<std::vector<cv::Point3f> > objpoints;

        // Creating vector to store vectors of 2D points for each checkerboard image
        std::vector<std::vector<cv::Point2f> > imgpoints;

        int img_rows_, img_cols_;
        int checkerboard_num_ = 7;
        int checkerboard_rows_ = 3;
        int checkerboard_cols_ = 5;
        int calibrate_count_ = 0;
        int calibrate_threshold_;
        bool calibrated_ = false;

        cv::Mat cameraMatrix, distCoeffs, optimal_mat;

        void cornersFromTop(const std::vector<cv::Point2f> &corners, std::vector<cv::Point2f> &corners_corrected);
        void calibrate();
        void reprojectionErrors();
};

}

#endif