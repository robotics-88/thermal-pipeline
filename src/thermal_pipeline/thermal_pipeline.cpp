/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/thermal_pipeline.h"

#include "opencv2/imgproc.hpp"

namespace thermal_pipeline
{
Thermal::Thermal()
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

int Thermal::thermalContours(const cv::Mat &img, const double min, const image_geometry::PinholeCameraModel model, std::vector<std::vector<cv::Point> > &contours) {
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
    cv::threshold(img, img, min, 255, cv::THRESH_TOZERO);
    
    // Find contours on the filtered image
    std::vector<std::vector<cv::Point> > all_contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours( img, all_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    // Filter contours by size
    double img_area = original.rows * original.cols;
    for ( size_t i = 0; i< all_contours.size(); i++ )
    {
        double area = cv::contourArea(all_contours.at(i));
        if ( area < 100 || (area / img_area) > 0.99) {
            continue;
        }
        contours_.push_back(all_contours.at(i));
    }
    contours = contours_;
    return all_contours.size();
}

void Thermal::contourCenters(const image_geometry::PinholeCameraModel model, std::vector<cv::Point> &centers, std::vector<cv::Point3d> &projected_centers) {
    for ( size_t i = 0; i< contours_.size(); i++ ) {
        cv::Moments m = cv::moments(contours_.at(i));
        cv::Point p(m.m10/m.m00, m.m01/m.m00);
        centers.push_back(p);
        // Projected
        cv::Point3d ray3d = model.projectPixelTo3dRay(centers.at(i));
        projected_centers.push_back(ray3d);
    }
}

void Thermal::projectContour(const image_geometry::PinholeCameraModel model, const std::vector<cv::Point> &contour, std::vector<cv::Point3d> &projected_contour) {
    for ( size_t i = 0; i< contour.size(); i++ ) {
        cv::Point3d ray3d = model.projectPixelTo3dRay(contour.at(i));
        projected_contour.push_back(ray3d);
    }
}

}