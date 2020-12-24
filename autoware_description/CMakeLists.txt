cmake_minimum_required(VERSION 3.0.2)
project(autoware_description)

find_package(catkin REQUIRED
  effort_controllers
  gazebo_ros_control
  position_controllers
  transmission_interface
  velocity_controllers
)

catkin_package(
  CATKIN_DEPENDS    
    effort_controllers
    gazebo_ros_control
    position_controllers
    transmission_interface
    velocity_controllers
)

install(DIRECTORY mesh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
