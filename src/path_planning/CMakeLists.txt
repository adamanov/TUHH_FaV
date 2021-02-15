cmake_minimum_required(VERSION 3.0.2)
project(path_planning)
add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs

)

find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include 
                            ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIR}
                            ${CMAKE_CURRENT_SOURCE_DIR})

add_message_files(
  FILES
  PathCoordinates.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES PathPlanning
#  CATKIN_DEPENDS gazebo_plugins gazebo_ros geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})
