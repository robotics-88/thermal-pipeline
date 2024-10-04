/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "calibration/calibrator.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto tm_node = std::make_shared<thermal_pipeline::Calibrator>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tm_node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}