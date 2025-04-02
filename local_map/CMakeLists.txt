cmake_minimum_required(VERSION 2.8.3)
project(local_map)

# # add_compile_options(-std=c++11)
# add_compile_options(-std=c++14)


# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   sensor_msgs
#   pcl_conversions
#   pcl_ros
#   nav_msgs
#   tf2_ros
#   tf2_eigen
# )

# # find_package(PCL 1.7 REQUIRED)
# find_package(PCL REQUIRED)

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
# )

# include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
# )
# link_directories(${PCL_LIBRARY_DIRS})

# # add_executable(${PROJECT_NAME}_node src/plane_ground_filter_node.cpp src/plane_ground_filter_core.cpp)
# # target_link_libraries(${PROJECT_NAME}_node
# #   ${catkin_LIBRARIES}
# #   ${PCL_LIBRARIES}
# # )


# ## 声明可执行文件
# add_executable(obstacle_perception_node src/obstacle_perception_node.cpp)
# ## 添加链接库，确保链接所有依赖的库
# target_link_libraries(obstacle_perception_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  message_filters
  
  PCL
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(obstacle_perception_node src/obstacle_perception_node.cpp)
## 添加链接库，确保链接所有依赖的库
target_link_libraries(obstacle_perception_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
