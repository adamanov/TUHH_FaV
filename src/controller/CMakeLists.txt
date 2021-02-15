cmake_minimum_required(VERSION 3.0.2)
project(controller)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
)


find_package(gazebo REQUIRED)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_message_files(
    FILES
    currentPose.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
 )

 




catkin_package(
#  INCLUDE_DIRS include
  DEPENDS 
      roscpp 
      gazebo_ros
      rospy
  CATKIN_DEPENDS 
      message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include 
                            ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIR}
                            ${CMAKE_CURRENT_SOURCE_DIR})


add_library(gazebo_pressure_plugin plugins/gazebo_pressure_plugin.cpp)
add_dependencies(gazebo_pressure_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_pressure_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


