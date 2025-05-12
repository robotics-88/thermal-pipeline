#include "thermal_pipeline/thermal_pipeline.h"

#include <gtest/gtest.h>

TEST(ColorConverter, convertColor) {
    ros::NodeHandle node;
    thermal_pipeline::Thermal thermal(node);

    cv::Mat img = cv::Mat::zeros(100, 100, CV_8UC3);
    thermal.convertToGray(img);
    int type = img.type();
    EXPECT_EQ(type, 0);
}

TEST(ThermalContour, thermalContour) {
    ros::NodeHandle node;
    thermal_pipeline::Thermal thermal(node);

    // Black image
    cv::Mat img = cv::Mat::zeros(100, 100, CV_8UC3);
    int startX = 25;
    int startY = 25;
    int endX = 75;
    int endY = 75;

    // Add a white filled rectangle in the center
    cv::rectangle(img, cv::Point(startX, startY), cv::Point(endX, endY), CV_RGB(255, 255, 255),
                  cv::FILLED);

    // Get contours
    thermal.convertToGray(img);
    cv::Mat contours_img;
    int num = thermal.thermalContours(img, contours_img);

    // Should be exactly 1
    EXPECT_EQ(num, 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}