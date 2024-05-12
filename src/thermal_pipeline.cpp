/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/thermal_pipeline.h"

namespace thermal_pipeline
{
Thermal::Thermal(ros::NodeHandle& node)
  : nh_(node)
  , private_nh_("~")
{
}

Thermal::~Thermal() {
}


}