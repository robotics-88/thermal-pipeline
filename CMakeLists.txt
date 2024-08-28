cmake_minimum_required(VERSION 3.5)
project(thermal_88)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

find_package(camera_calibration_parsers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(messages_88 REQUIRED)
find_package(rviz2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###########
## Build ##
###########
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(thermal_wrapper_node src/thermal_pipeline/thermal_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ library
add_library(thermal_pipeline SHARED
  src/thermal_pipeline/image_annotation.cpp
  src/thermal_pipeline/thermal_pipeline_wrapper.cpp
  src/thermal_pipeline/thermal_pipeline.cpp
  src/thermal_pipeline/hotspot_tracker.cpp
)

ament_target_dependencies(thermal_pipeline 
  cv_bridge
  image_geometry
  geometry_msgs
  message_filters
  messages_88
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
)

## Specify libraries to link a library or executable target against
target_link_libraries(thermal_wrapper_node
    thermal_pipeline
    ${OpenCV_LIBRARIES}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(thermal_calibrator_node src/calibration/calibrator_node.cpp)

## Declare a C++ library
add_library(thermal_calibrator SHARED
  src/calibration/calibrator.cpp
)

ament_target_dependencies(thermal_calibrator 
  cv_bridge
  image_geometry
  rclcpp
  std_msgs
  sensor_msgs
)

## Specify libraries to link a library or executable target against
target_link_libraries(thermal_calibrator_node
    thermal_calibrator
    ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  thermal_pipeline
  thermal_wrapper_node
  thermal_calibrator
  thermal_calibrator_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  
  find_package(ament_cmake_gtest REQUIRED)

#   # test all launch files
#   roslaunch_add_file_check(launch)

#   find_package(rostest REQUIRED)

#   add_rostest_gtest(tests_thermal
#     launch/test/tests_thermal.launch
#     src/thermal_pipeline/thermal_pipeline.cpp
#     test/tests_thermal_pipeline.cpp
#   )
#   target_link_libraries(tests_thermal 
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBRARIES}
#   )
endif()

ament_package()
