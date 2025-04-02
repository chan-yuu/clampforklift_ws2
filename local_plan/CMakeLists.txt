cmake_minimum_required(VERSION 3.10)
project(local_plan)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# catkin
find_package(
    catkin REQUIRED COMPONENTS
    car_interfaces
    nav_msgs
    roscpp
    geometry_msgs
    message_generation
    message_runtime
    std_msgs
    rospy
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS car_interfaces geometry_msgs message_generation message_runtime nav_msgs roscpp rospy std_msgs 
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(
    PATH_SEARCHER SHARED
    src/rs_path.cpp
    src/costmap_subscriber.cpp
    src/goal_pose_subscriber.cpp
    src/init_pose_subscriber.cpp
    src/hybrid_a_star.cpp
    src/hybrid_a_star_flow.cpp
)

target_link_libraries(
    PATH_SEARCHER
    ${catkin_LIBRARIES}
)

add_executable(local_plan src/run_hybrid_astar.cpp)
target_link_libraries(local_plan PATH_SEARCHER)
add_dependencies(local_plan car_interfaces_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
