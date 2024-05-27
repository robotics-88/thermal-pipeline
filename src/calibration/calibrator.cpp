/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include "thermal_pipeline/calibrator.h"

namespace thermal_pipeline
{
Calibrator::Calibrator(ros::NodeHandle& node)
  : nh_(node)
  , private_nh_("~")
  , calibrate_threshold_(100)
{
    // Params
    private_nh_.param("checkerboard_squares", checkerboard_num_, checkerboard_num_);
    private_nh_.param("min_checkerboard", calibrate_threshold_, calibrate_threshold_);
    std::string camera_name;
    private_nh_.param("camera_name", camera_name, camera_name);

    // ROS setup
    image_chessboard_pub_ = nh_.advertise<sensor_msgs::Image>(camera_name + "/image_chessboard", 10);
    image_rect_pub_ = nh_.advertise<sensor_msgs::Image>(camera_name + "/image_rect", 10);
    // info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name + "/camera_info", 10);
    std::string sub_topic = camera_name + "/image_raw";
    image_sub_ = nh_.subscribe<sensor_msgs::Image>(sub_topic, 10, &Calibrator::imageCallback, this);
}

Calibrator::~Calibrator() {
}

void Calibrator::imageCallback(const sensor_msgs::Image::ConstPtr &image) {
    if (first_image_) {
        img_rows_ = image->height;
        img_cols_ = image->width;
        first_image_ = false;
    }

    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGRA8); 
    cv::Mat mat_thermal;
    mat_thermal = cv_ptr->image;
    cv::cvtColor(mat_thermal, mat_thermal, cv::COLOR_BGRA2GRAY);

    cv::equalizeHist(mat_thermal, mat_thermal); // TODO, is this helping or hurting? improves ability to detect the chessboard, but does it make calibration worse?

    if (calibrated_) {
        // TODO something weird is happening in undistort, compare with ROS calibrator
        cv::Mat rect;
        cv::Mat mapx, mapy;
        // mat_thermal.convertTo(mat_thermal, CV_32FC1);
        // cv::undistort(mat_thermal, rect, cameraMatrix, distCoeffs);
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat::eye(cv::Size(3,3), CV_64FC1), optimal_mat, cv::Size(img_cols_, img_rows_), CV_32FC1, mapx, mapy);
        cv::remap(mat_thermal, rect, mapx, mapy, CV_INTER_LINEAR);
        cv_bridge::CvImage image_rect_msg;
        image_rect_msg.header   = image->header; // Same timestamp and tf frame as input image
        image_rect_msg.encoding = sensor_msgs::image_encodings::MONO8;
        image_rect_msg.image    = rect;
        image_rect_pub_.publish(image_rect_msg.toImageMsg());

        return;
    }


    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<checkerboard_rows_; i++)
    {
        for(int j{0}; j<checkerboard_cols_; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }
    
    
    cv::Mat frame;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts, corrected_corners;
    bool success;
    
    cv::cvtColor(mat_thermal,frame,cv::COLOR_GRAY2BGR);

    // Finding checker board corners
    // If desired number of corners are found in the image then success = true  
    success = cv::findChessboardCorners(frame, cv::Size(checkerboard_cols_, checkerboard_rows_), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH);
    
    /* 
    * If desired number of corner are detected,
    * we refine the pixel coordinates and display 
    * them on the images of checker board
    */
    if(success)
    {
        
        // refining pixel coordinates for given 2d points.
        cv::cornerSubPix(mat_thermal, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001));
        
        // Displaying the detected corner points on the checker board
        cv::drawChessboardCorners(frame, cv::Size(checkerboard_cols_, checkerboard_rows_), corner_pts, success);
        
        objpoints.push_back(objp);
        imgpoints.push_back(corner_pts);
        calibrate_count_++;
        float percent_complete = 100.0 * ((float) calibrate_count_ )/ ((float) calibrate_threshold_);
        int perc = (int) percent_complete;
        if (perc % 10 == 0) {
            std::cout << perc << " percent calibrated..." << std::endl;
        }
    }
    cv_bridge::CvImage chessboard_align_msg;
    chessboard_align_msg.header   = image->header; // Same timestamp and tf frame as input image
    chessboard_align_msg.encoding = sensor_msgs::image_encodings::BGR8;
    chessboard_align_msg.image    = frame;
    image_chessboard_pub_.publish(chessboard_align_msg.toImageMsg());

    if (calibrate_count_ > calibrate_threshold_) {
        if (!calibrated_) {
            calibrate();
        }
    }
}

void Calibrator::calibrate() {
    /*
    * Performing camera calibration by 
    * passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the 
    * detected corners (imgpoints)
    */
    cv::Mat R,T;
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(img_cols_, img_rows_), cameraMatrix, distCoeffs, R, T);
    
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;

    optimal_mat = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(img_cols_, img_rows_), 0, cv::Size(img_cols_, img_rows_));
    std::cout << "P matrix : " << optimal_mat << std::endl;
    calibrated_ = true;
}

// TODO if use custom calibrator, need to compute error
// void Calibrator::reprojectionErrors() {
//     std::vector<cv::Point2f> imagePoints2;
//     size_t totalPoints = 0;
//     double totalErr = 0, err;
//     perViewErrors.resize(objpoints.size());
    
//     for(size_t i = 0; i < objpoints.size(); ++i )
//     {
//     // if (fisheye)
//     // {
//     // fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
//     // distCoeffs);
//     // }
//     // else
//     // {
//     cv::projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
//     // }
//     err = norm(imgpoints[i], imagePoints2, cv::NORM_L2);
//  }
}