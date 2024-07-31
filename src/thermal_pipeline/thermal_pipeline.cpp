/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/thermal_pipeline.h"

#include "opencv2/imgproc.hpp"

namespace thermal_pipeline
{
Thermal::Thermal(ros::NodeHandle& node)
    : nh_(node)
    , private_nh_("~")
    , camera_model_set_(false)
{
}

Thermal::~Thermal() {
}
void Thermal::convertToGray(cv::Mat &img) {
    int type = img.type();
    if (type == CV_8UC3) {
        // BGR
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
    else if (type == CV_8UC4) {
        // BGRA
        cv::cvtColor(img, img, CV_BGRA2GRAY);
    }
}

int Thermal::thermalContours(const cv::Mat &img, cv::Mat &img_contours, double min, double max) {
    contours_.clear();
    // Save a copy of the unedited image
    cv::Mat original = img.clone();

    // Dilate and erode to cluster hot spots
    int erosion_size = 3;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
            cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
            cv::Point( erosion_size, erosion_size ) );
    cv::dilate(img, img, element);
    cv::erode(img, img, element);

    // Apply tresholding to remove low temperatures
    double threshold = 200;
    cv::threshold(img, img, min, max, cv::THRESH_TOZERO);
    
    // Find contours on the filtered image
    std::vector<std::vector<cv::Point> > all_contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours( img, all_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    // Draw contours on the output image
    cv::cvtColor(original, img_contours, cv::COLOR_GRAY2BGR);
    cv::Scalar color = cv::Scalar( 0, 0, 255 );
    double img_area = original.rows * original.cols;
    for ( size_t i = 0; i< all_contours.size(); i++ )
    {
        double area = cv::contourArea(all_contours.at(i));
        if ( area < 100 || (area / img_area) > 0.99) {
            continue;
        }
        cv::drawContours( img_contours, all_contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
        contours_.push_back(all_contours.at(i));
    }
    return all_contours.size();
}

void Thermal::contourCenters(const sensor_msgs::CameraInfo &info, std::vector<cv::Point> &centers, std::vector<cv::Point3d> &projected_centers) {
    if (!camera_model_set_) {
        camera_model_.fromCameraInfo(info);
    }

    for ( size_t i = 0; i< contours_.size(); i++ ) {
        cv::Moments m = cv::moments(contours_.at(i));
        cv::Point p(m.m10/m.m00, m.m01/m.m00);
        centers.push_back(p);
        // Projected
        cv::Point3d ray3d = camera_model_.projectPixelTo3dRay(centers.at(i));
        projected_centers.push_back(ray3d);
    }
}

// void Thermal::projectedCenters(const std::vector<cv::Point> &centers, const sensor_msgs::CameraInfo &info, const geometry_msgs::TransformStamped &transform_image2map, const std_msgs::Header &header, std::vector<cv::Point> &projected_centers) {
//     if (!camera_model_set_) {
//         camera_model_.fromCameraInfo(info);
//     }
//     for ( size_t i = 0; i< centers.size(); i++ ) {
//         cv::Point3d ray3d = camera_model_.projectPixelTo3dRay(centers.at(i));
//         cv::Point p;
//         projected_centers.push_back(p);
//     }
// }

}