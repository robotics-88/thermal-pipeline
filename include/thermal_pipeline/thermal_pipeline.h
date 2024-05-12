/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef THERMAL_PIPELINE_H_
#define THERMAL_PIPELINE_H_

#include <string>

#include <ros/ros.h>

namespace thermal_pipeline {
/**
 * @class SpeciesMapper
 * @brief A class for converting images with bounding boxes on detected species into maps
 */
class Thermal {
    public:
        Thermal(ros::NodeHandle& node);

        ~Thermal();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
};

}

#endif