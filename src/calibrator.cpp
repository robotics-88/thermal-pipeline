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
{
    // Params
    private_nh_.param("checkerboard_squares", checkerboard_num_, checkerboard_num_);
    std::string camera_name;
    private_nh_.param("camera_name", camera_name, camera_name);

    // ROS setup
    image_chessboard_pub_ = nh_.advertise<sensor_msgs::Image>(camera_name + "/image_chessboard", 10);
    // info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name + "/camera_info", 10);
    std::string sub_topic = camera_name + "/image_raw";
    image_sub_ = nh_.subscribe<sensor_msgs::Image>(sub_topic, 10, &Calibrator::imageCallback, this);
}

Calibrator::~Calibrator() {
    /*
    * Performing camera calibration by 
    * passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the 
    * detected corners (imgpoints)
    */
    cv::Mat cameraMatrix,distCoeffs,R,T;
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(img_rows_, img_cols_), cameraMatrix, distCoeffs, R, T);
    
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    std::cout << "Rotation vector : " << R << std::endl;
    std::cout << "Translation vector : " << T << std::endl;
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
    // Convert float to 
    // mat_float.convertTo(mat_float, CV_8UC1);
    // double min, max;
    // cv::minMaxLoc(mat_float, &min, &max);
    // std::cout << "min: " << min << ", max: " << max << std::endl;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<checkerboard_num_; i++)
    {
        for(int j{0}; j<checkerboard_num_; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }
    
    
    cv::Mat frame = mat_float;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts;
    bool success;
    
    // cv::cvtColor(mat_float,frame,cv::COLOR_GRAY2BGR);

    // Finding checker board corners
    // If desired number of corners are found in the image then success = true  
    success = cv::findChessboardCorners(mat_float, cv::Size(checkerboard_num_, checkerboard_num_), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
    std::cout << "chessboard corner success? " << success << std::endl;
    
    /* 
    * If desired number of corner are detected,
    * we refine the pixel coordinates and display 
    * them on the images of checker board
    */
    if(success)
    {
        cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
        
        // refining pixel coordinates for given 2d points.
        cv::cornerSubPix(mat_float,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);
        
        // Displaying the detected corner points on the checker board
        cv::drawChessboardCorners(frame, cv::Size(checkerboard_num_, checkerboard_num_), corner_pts, success);
        
        objpoints.push_back(objp);
        imgpoints.push_back(corner_pts);
    }
    cv_bridge::CvImage chessboard_align_msg;
    chessboard_align_msg.header   = image->header; // Same timestamp and tf frame as input image
    chessboard_align_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC4;
    chessboard_align_msg.image    = frame;
    image_chessboard_pub_.publish(chessboard_align_msg.toImageMsg());
}

}