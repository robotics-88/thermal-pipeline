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

int Thermal::thermalContours(const cv::Mat &img, cv::Mat &img_contours) {
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
    double threshold = 220;
    cv::threshold(img, img, threshold, 255, cv::THRESH_TOZERO);
    
    // Find contours on the filtered image
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours( img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    // Draw contours on the output image
    cv::cvtColor(original, img_contours, cv::COLOR_GRAY2BGR);
    cv::Scalar color = cv::Scalar( 0, 0, 255 );
    double img_area = original.rows * original.cols;
    for( size_t i = 0; i< contours.size(); i++ )
    {
        double area = cv::contourArea(contours.at(i));
        std::cout << "area percent " << (area / img_area) << std::endl;
        if ( area < 100 || (area / img_area) > 0.99) {
            continue;
        }
        cv::drawContours( img_contours, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
    }
    return contours.size();
}

}