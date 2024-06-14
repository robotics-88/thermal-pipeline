/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/thermal_pipeline_wrapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thermal_pipeline");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node;
  thermal_pipeline::ThermalWrapper thermal(node);

  ros::spin();

  return 0;
}