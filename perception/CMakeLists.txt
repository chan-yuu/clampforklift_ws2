cmake_minimum_required(VERSION 3.0.2)
project(perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  car_interfaces
  geometry_msgs
  message_generation
  message_runtime
  tf2_ros
  roscpp
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  tf2_geometry_msgs
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS geometry_msgs message_generation message_runtime car_interfaces roscpp rospy std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(gps_imu_pub src/gps_imu.cpp)
target_link_libraries(gps_imu_pub
  ${catkin_LIBRARIES}
)
add_dependencies(gps_imu_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} car_interfaces_generate_messages_cpp)


add_executable(gps_sim_pub src/gps_sim.cpp)
target_link_libraries(gps_sim_pub
  ${catkin_LIBRARIES}
)
add_dependencies(gps_sim_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} car_interfaces_generate_messages_cpp)


add_executable(tf_odom_publisher src/tf_odom_publisher.cpp)
target_link_libraries(tf_odom_publisher ${catkin_LIBRARIES})
add_dependencies(tf_odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} car_interfaces_generate_messages_cpp)


add_executable(odom_pub src/tf_tree_odom_pub.cpp)
target_link_libraries(odom_pub ${catkin_LIBRARIES})
add_dependencies(odom_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} car_interfaces_generate_messages_cpp)

# add_executable(gps_odom src/gps_odom.cpp)
# target_link_libraries(gps_odom ${catkin_LIBRARIES})
# add_dependencies(gps_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} car_interfaces_generate_messages_cpp)


# gazebo仿真中的tf发布：
add_executable(gazebo_tf src/tf_pub_by_odom.cpp)
target_link_libraries(gazebo_tf ${catkin_LIBRARIES})


add_executable(tf_odom_publisher_sim src/tf_odom_publisher_sim.cpp)
target_link_libraries(tf_odom_publisher_sim ${catkin_LIBRARIES})