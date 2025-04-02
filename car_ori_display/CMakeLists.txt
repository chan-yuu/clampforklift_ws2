cmake_minimum_required(VERSION 3.0.2)
project(car_ori_display)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  tf
  message_generation
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  geometry_msgs
  visualization_msgs
  angles
  car_interfaces

  roscpp
  rviz
  std_msgs
  geometry_msgs
  std_srvs
  geodesy
  geographic_msgs
)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
# find_package(GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED)


## 声明消息文件
# add_message_files(
#   FILES
#   CarOriInterface.msg
#   PathSpeedCtrlInterface.msg
#   FaultDiagnosisInterface.msg
#   GpsImuInterface.msg
# )

## 生成消息
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

## catkin_package 必须在 generate_messages 之后调用
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs visualization_msgs angles rviz std_msgs tf message_runtime sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

qt5_wrap_cpp(MOC_FILES
  include/car_ori_display/angle_display.h
  include/car_ori_display/car_ori_display.h
  include/car_ori_display/path_speed_ctrl_display.h
  include/car_ori_display/save_bag.h
  include/car_ori_display/button_rviz.h
  include/car_ori_display/button_rviz2.h
  include/car_ori_display/diagnosis_panel.h
  # include/car_ori_display/calibration_panel.h
  include/car_ori_display/calibration_lidar_panel.h
  include/car_ori_display/points_manager_panel.h
  include/car_ori_display/bag_recorder_panel.h
  include/car_ori_display/gps_imu_display.h
  include/car_ori_display/lola_to_utm_panel.h

  include/car_ori_display/service_config_panel.h
)

## 确保在编译库和可执行文件之前生成消息头文件
add_library(car_ori_display
  src/angle_display.cpp
  src/car_ori_display.cpp
  src/path_speed_ctrl_display.cpp
  src/save_bag.cpp
  src/button_rviz.cpp
  src/button_rviz2.cpp
  src/diagnosis_panel.cpp
  # src/calibration_panel.cpp
  src/calibration_lidar_panel.cpp
  src/points_manager_panel.cpp
  src/bag_recorder_panel.cpp
  src/gps_imu_display.cpp  
  src/lola_to_utm_panel.cpp

  src/service_config_panel.cpp
  ${MOC_FILES}
)

target_link_libraries(car_ori_display
  ${catkin_LIBRARIES}
  Qt5::Widgets
  yaml-cpp
  ${OpenCV_LIBS}
  # Geographic
)
target_include_directories(car_ori_display PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS car_ori_display
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# add_executable(angle_publisher src/angle_publisher.cpp)
# target_link_libraries(angle_publisher
#   ${catkin_LIBRARIES}
# )

# add_executable(model_node src/model.cpp)
# target_link_libraries(model_node
#   ${catkin_LIBRARIES}
# )

# add_executable(lidar_calibration_node src/lidar_calibration_node.cpp)
# target_link_libraries(lidar_calibration_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

add_compile_options(-fPIC)
# add_executable(multi_lidar_calibration src/multi_lidar_calibration.cpp)
# target_link_libraries(multi_lidar_calibration
#   ${catkin_LIBRARIES}
# )
