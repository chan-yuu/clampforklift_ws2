cmake_minimum_required(VERSION 3.0.2)
project(v2n)

find_package(catkin REQUIRED COMPONENTS
  car_interfaces
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  roslib
  geometry_msgs
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)


include_directories(
  include
 ${catkin_INCLUDE_DIRS}
 ${mosquitto_INCLUDE_DIRS}
)

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    link_directories(/usr/lib/x86_64-linux-gnu)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    link_directories(/usr/lib/aarch64-linux-gnu)
endif()

add_executable(sub_cloud src/sub_cloud.cpp)
# 修改.so文件位置 sudo apt-get install libmosquitto-dev
# target_link_libraries(test_node ${catkin_LIBRARIES} /usr/lib/x86_64-linux-gnu/libmosquitto.so)
# target_link_libraries(test_node ${catkin_LIBRARIES} /usr/lib/aarch64-linux-gnu/libmosquitto.so)
target_link_libraries(sub_cloud
    ${catkin_LIBRARIES}
    mosquitto
)

add_executable(pub_cloud src/pub_cloud.cpp)
# 需要安装对应的库，详情见install_paho.sh
target_link_libraries(pub_cloud ${catkin_LIBRARIES} paho-mqttpp3 paho-mqtt3as)