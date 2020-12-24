cmake_minimum_required(VERSION 3.0.2)
project(autoware_base)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  gazebo_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES autoware_base
  CATKIN_DEPENDS 
    ackermann_msgs 
    gazebo_msgs 
    geometry_msgs 
    roscpp 
    sensor_msgs 
    std_msgs
    tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(autoware_base src/autoware_base.cpp)
target_link_libraries(autoware_base ${catkin_LIBRARIES})