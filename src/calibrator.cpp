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
  , calibrate_threshold_(25)
{
    // Params
    private_nh_.param("checkerboard_squares", checkerboard_num_, checkerboard_num_);
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
    cv::Mat mat_float;
    mat_float = cv_ptr->image;
    cv::cvtColor(mat_float,mat_float,cv::COLOR_BGRA2GRAY);

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<checkerboard_num_; i++)
    {
        for(int j{0}; j<checkerboard_num_; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }
    
    
    cv::Mat frame = mat_float;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts, corrected_corners;
    bool success;
    
    cv::cvtColor(mat_float,frame,cv::COLOR_GRAY2BGR);

    // Finding checker board corners
    // If desired number of corners are found in the image then success = true  
    success = cv::findChessboardCorners(mat_float, cv::Size(checkerboard_num_, checkerboard_num_), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    
    /* 
    * If desired number of corner are detected,
    * we refine the pixel coordinates and display 
    * them on the images of checker board
    */
    if(success)
    {
        cornersFromTop(corner_pts, corrected_corners);
        
        // refining pixel coordinates for given 2d points.
        cv::cornerSubPix(mat_float, corrected_corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        
        // Displaying the detected corner points on the checker board
        cv::drawChessboardCorners(frame, cv::Size(checkerboard_num_, checkerboard_num_), corrected_corners, success);
        
        objpoints.push_back(objp);
        imgpoints.push_back(corrected_corners);
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
        else {
            // TODO something weird is happening in undistort, compare with ROS calibrator
            cv::Mat rect;
            cv::Mat mapx, mapy;
            cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat::eye(cv::Size(3,3), CV_64FC1), optimal_mat, cv::Size(img_rows_, img_cols_), CV_32FC1, mapx, mapy);
            cv::remap(mat_float, rect, mapx, mapy, CV_INTER_LINEAR);
            cv_bridge::CvImage image_rect_msg;
            image_rect_msg.header   = image->header; // Same timestamp and tf frame as input image
            image_rect_msg.encoding = sensor_msgs::image_encodings::MONO8;
            image_rect_msg.image    = rect;
            image_rect_pub_.publish(image_rect_msg.toImageMsg());
        }
    }
}

void Calibrator::cornersFromTop(const std::vector<cv::Point2f> &corners, std::vector<cv::Point2f> &corners_corrected) {
    cv::Point2f first_corner = corners.at(0);
    cv::Point2f last_corner = corners.at(corners.size() - 1);

    std::vector<bool> direction_corners;
    direction_corners.push_back((last_corner.x - first_corner.x) > 0);
    direction_corners.push_back((last_corner.y - first_corner.y) > 0);

    cv::Mat corner_mat(corners);
    corner_mat = corner_mat.reshape(2, checkerboard_num_);
    if (direction_corners.at(0) && direction_corners.at(1)) {
        // Already good
        corners_corrected = corners;
        return;
    }
    else if (!direction_corners.at(0) && !direction_corners.at(1)) {
        cv::flip(corner_mat, corner_mat, 1);
    }
    else if (direction_corners.at(0)) {
        cv::rotate(corner_mat, corner_mat, cv::ROTATE_90_CLOCKWISE);
    }
    else {
        cv::rotate(corner_mat, corner_mat, cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    // retrieve corners from mat
    cv::Mat xchan, ychan;
    std::vector<cv::Mat> channels(2);
    // split img:
    split(corner_mat, channels);
    xchan = channels.at(0);
    ychan = channels.at(1);
    for (int rr = 0; rr < corner_mat.rows; rr++) {
        for (int cc = 0; cc < corner_mat.cols; cc++) {
            cv::Point2f point(xchan.at<float>(cc,rr), ychan.at<float>(cc,rr));
            corners_corrected.push_back(point);
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
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(img_rows_, img_cols_), cameraMatrix, distCoeffs, R, T);
    
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    // std::cout << "Rotation vector : " << R << std::endl;
    // std::cout << "Translation vector : " << T << std::endl;

    optimal_mat = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(img_rows_, img_cols_), 0);
    // cv::undistort()
    std::cout << "P matrix : " << optimal_mat << std::endl;
    calibrated_ = true;
}

}