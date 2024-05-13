/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "thermal_pipeline/calibrator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thermal_calibrator");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node;
  thermal_pipeline::Calibrator calibrator(node);

  ros::spin();

  return 0;
}