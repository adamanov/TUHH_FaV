cmake_minimum_required(VERSION 3.0.2)
project(object_detection)


find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  geometry_msgs
  std_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
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
#  LIBRARIES range_sensor
#  CATKIN_DEPENDS apriltag_ros geometry_msgs std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
