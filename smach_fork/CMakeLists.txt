cmake_minimum_required(VERSION 3.0.2)
project(smach_fork)
# 在文件头部加入 find_package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  car_interfaces
  geometry_msgs
  rospy
  nav_msgs
  message_runtime
  message_generation
  tf
)

# 配置 catkin 包
catkin_package(
  CATKIN_DEPENDS std_msgs roscpp
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)