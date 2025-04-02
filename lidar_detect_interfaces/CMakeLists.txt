cmake_minimum_required(VERSION 3.0.2)
project(lidar_detect_interfaces)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
)

add_message_files(
  FILES
  # add lidar detect
  Centroids.msg
  CloudCluster.msg
  CloudClusterArray.msg
  DetectedObject.msg
  DetectedObjectArray.msg
  BoundingBox.msg
  LaneArray.msg
  Lane.msg
  Waypoint.msg
  WaypointState.msg
  DTLane.msg
  PolygonArray.msg
  LidarDetectObject.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_msgs
 CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
