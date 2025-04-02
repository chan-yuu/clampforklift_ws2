cmake_minimum_required(VERSION 3.0.2)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  car_interfaces
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs 
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  # control/stanley_control.h
  ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIRS})

# add_executable(pid_controller src/pid_control.cpp)
# target_link_libraries(pid_controller ${catkin_LIBRARIES})

#add_executable(stanley_controller src/stanley_control.cpp)
#target_link_libraries(stanley_controller ${catkin_LIBRARIES})

add_executable(Control_FSM src/Control_FSM.cpp src/stanley_control.cpp src/pid_control.cpp)
target_link_libraries(Control_FSM ${catkin_LIBRARIES})
add_dependencies(Control_FSM ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} car_interfaces_generate_messages_cpp)

add_executable(can_pub src/can_pub.cpp)
target_link_libraries(can_pub
  ${catkin_LIBRARIES}
)

# install(TARGETS
#   pid_controller
#   stanley_controller
#   mpc_controller
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# catkin_package()
add_executable(can_feedback src/can_feedback_pub.cpp src/can_feedback_main.cpp)
target_link_libraries(can_feedback
  ${catkin_LIBRARIES}
)
add_dependencies(can_feedback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
