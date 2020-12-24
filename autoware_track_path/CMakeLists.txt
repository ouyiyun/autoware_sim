cmake_minimum_required(VERSION 3.0.2)
project(autoware_track_path)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES autoware_track_path
  CATKIN_DEPENDS 
    ackermann_msgs 
    geometry_msgs 
    nav_msgs 
    roscpp 
    tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})