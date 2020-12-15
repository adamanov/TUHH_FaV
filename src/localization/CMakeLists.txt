cmake_minimum_required(VERSION 3.0.2)
project(localization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  message_generation
)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
  
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)
