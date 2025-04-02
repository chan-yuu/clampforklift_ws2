#!/bin/bash

# 定义颜色
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'

NC='\033[0m' # No Color


slow_print() {
    text="$1"
    delay="$2"
    for (( i=0; i<${#text}; i++ )); do
        char="${text:$i:1}"
        echo -n "$char"
        sleep "$delay"
    done
    echo
}

welcome_msg="===============================================================================
======欢迎使用cyun的一键安装工具，人生苦短，三省吾身，省时省力省心!=======
===============================================================================

                        .-~~~~~~~~~-._       _.-~~~~~~~~~-.
                    __.'              ~.   .~              \`.__
                .'//     开卷有益        \./     书山有路     \ \`.
                .'//   不积跬步,无以至千里  |  不积小流,无以成江海 \ \`.
            .'// .-~~~~~~~~~~~~~~-._     |     _,-~~~~~~~~~~~. \`.
            .'//.-\"                 \`-.  |  .-'                 \"-.\`.
        .'//______.============-..   \ | /   ..-============.______\`.
        .'______________________________\|/______________________________\`
        ----------------------------------------------------------------------"

# 调用函数逐行输出欢迎信息
slow_print "$welcome_msg" 0.001

echo -e "${GREEN}************************************************************************${NC}"
echo -e "${GREEN}*                                                                      *${NC}"
echo -e "${GREEN}*  ${CYAN}                  ██████╗ ██╗   ██╗██████╗ ████████╗               ${GREEN} *${NC}"
echo -e "${GREEN}*  ${CYAN}                  ██╔══██╗██║   ██║██╔══██╗╚══██╔══╝               ${GREEN} *${NC}"
echo -e "${GREEN}*  ${CYAN}                  ██████╔╝██║   ██║██████╔╝   ██║                  ${GREEN} *${NC}"
echo -e "${GREEN}*  ${CYAN}                  ██╔═══╝ ██║   ██║██╔══██╗   ██║                  ${GREEN} *${NC}"
echo -e "${GREEN}*  ${CYAN}                  ██║     ╚██████╔╝██║  ██║   ██║                  ${GREEN} *${NC}"
echo -e "${GREEN}*  ${CYAN}                  ╚═╝      ╚═════╝ ╚═╝  ╚═╝   ╚═╝                  ${GREEN} *${NC}"
echo -e "${GREEN}*                                                                      *${NC}"
echo -e "${GREEN}*  ${YELLOW}-----------------------------------------------------------${NC}"
echo -e "${GREEN}*  ${YELLOW}---------- Designed by CYUN 2025.3.26 ------------v3.1-----${NC}"
echo -e "${GREEN}*  ${YELLOW}-----------------------------------------------------------${NC}"
echo -e "${GREEN}*                                                                      *${NC}"
echo -e "${GREEN}************************************************************************${NC}"

sudo apt update
echo -e "INFO： 本脚本用于一键安装叉车的仿真环境需要的依赖等"
echo -e "INFO： 回车继续安装"
read -n 1

#===============================git 依赖库===============================
# 创建安装目录（假设安装到当前目录的 local_install 文件夹下，可自行修改）
install_dir="lib"
if [ ! -d "$install_dir" ]; then
    mkdir -p "$install_dir"
fi

cd $install_dir

git clone https://github.com/geographiclib/geographiclib.git
cd geographiclib
mkdir build
cd build
cmake ..
make
sudo make install
# 返回到初始目录
cd ../..
echo "GeographicLib 已成功安装到当前目录的 lib 文件夹下。"


# 克隆 paho.mqtt.cpp 仓库
git clone https://github.com/eclipse-paho/paho.mqtt.cpp.git
cd paho.mqtt.cpp

# 创建并进入 build 目录
mkdir build
cd build
cmake ..
make
sudo make install
# 返回到上级目录
cd ../..
# 克隆 paho.mqtt.c 仓库
git clone https://github.com/eclipse-paho/paho.mqtt.c.git
cd paho.mqtt.c
# 创建并进入 build 目录
mkdir build
cd build
cmake ..
make
sudo make install
# 返回到初始目录
cd ../..
echo "paho.mqtt.cpp 和 paho.mqtt.c 已成功安装到 $install_dir 文件夹下。"

git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout tags/4.2.0-ros
# 创建并进入 build 目录
mkdir build
cd build
cmake ..
make
sudo make install
# 返回到初始目录
cd ../..
echo "gtsam 已成功安装到 $install_dir 文件夹下。"
#===============================git 依赖库===============================


sudo apt-get install -y python-yaml
sudo apt install -y libgsettings-qt1
sudo apt install -y python3-virtualenv python3-pip
sudo apt-get -y install libgflags-dev
sudo apt install -y ros-$ROS_DISTRO-nmea-msgs
sudo apt install -y ros-$ROS_DISTRO-jsk-topic-tools
sudo apt install -y ros-$ROS_DISTRO-grid-map-ros
sudo apt install -y ros-$ROS_DISTRO-jsk-rviz-plugins
sudo apt install -y ros-$ROS_DISTRO-image-view2
sudo apt install -y ros-$ROS_DISTRO-jsk-recognition-msgs
sudo apt install -y ros-$ROS_DISTRO-gps-common
sudo apt install -y ros-$ROS_DISTRO-velodyne-gazebo-plugins
sudo apt install -y ros-$ROS_DISTRO-velodyne-pointcloud
sudo apt install -y ros-$ROS_DISTRO-nmea-navsat-driver
sudo apt install -y ros-$ROS_DISTRO-lgsvl-msgs
sudo apt install -y ros-$ROS_DISTRO-geodesy
sudo apt install -y ros-$ROS_DISTRO-rosbridge-server
sudo apt install -y ros-$ROS_DISTRO-sound-play
sudo apt install -y ros-$ROS_DISTRO-automotive-platform-msgs
sudo apt install -y ros-$ROS_DISTRO-gscam
sudo apt install -y ros-$ROS_DISTRO-qpoases-vendor
sudo apt install -y ros-$ROS_DISTRO-imu-tools
sudo apt install -y ros-$ROS_DISTRO-automotive-platform-msgs
sudo apt install -y ros-$ROS_DISTRO-velodyne
sudo apt install -y ros-$ROS_DISTRO-velocity-controllers
sudo apt install -y ros-$ROS_DISTRO-carla*
sudo apt install -y ros-$ROS_DISTRO-velodyne-description
sudo apt install -y ros-$ROS_DISTRO-automotive-navigation-msgs
sudo apt install -y ros-$ROS_DISTRO-uvc-camera
sudo apt install -y ros-$ROS_DISTRO-effort-controllers
sudo apt install -y ros-$ROS_DISTRO-ackermann*
sudo apt install -y ros-$ROS_DISTRO-derived-object*
sudo apt install  -y libomp5
sudo apt install -y libvulkan-dev vulkan-tools
sudo apt-get -y install libepoxy-dev

sudo apt-get install -y python3-numpy
sudo apt-get install -y python3-numpy-dev

sudo apt install -y ros-$ROS_DISTRO-navigation
sudo apt install -y  ros-$ROS_DISTRO-joy
sudo apt install -y  ros-$ROS_DISTRO-gazebo-ros-control
sudo apt install -y  ros-$ROS_DISTRO-joint-state-controller
sudo apt install -y  ros-$ROS_DISTRO-position-controllers
sudo apt install -y  ros-$ROS_DISTRO-effort-controllers
sudo apt install -y  ros-$ROS_DISTRO-cv-bridge
sudo apt install -y  ros-$ROS_DISTRO-controller-manager
sudo apt install -y  ros-$ROS_DISTRO-hector-mapping
sudo apt install -y  ros-$ROS_DISTRO-gmapping
sudo apt install -y ros-$ROS_DISTRO-plotjuggler
sudo apt install -y ros-$ROS_DISTRO-plotjuggler-ros
sudo apt install -y libdw-dev
sudo apt install -y ros-$ROS_DISTRO-osqp-vendor

sudo apt install -y  ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt install -y ros-$ROS_DISTRO-driver-base
sudo apt install -y ros-$ROS_DISTRO-ackermann-msgs
sudo apt install -y ros-$ROS_DISTRO-rtabmap-ros
sudo apt install -y ros-$ROS_DISTRO-ros-controllers
sudo apt install -y ros-$ROS_DISTRO-map-server
sudo apt install -y  ros-$ROS_DISTRO-amcl
sudo apt install -y  ros-$ROS_DISTRO-move-base
sudo apt install -y  ros-$ROS_DISTRO-nav-core
sudo apt install -y  ros-$ROS_DISTRO-costmap-*
sudo apt install -y  ros-$ROS_DISTRO-teb-local-planner
sudo apt install -y  ros-$ROS_DISTRO-global-planner
sudo apt-get install -y ros-$ROS_DISTRO-usb-cam
sudo apt-get install -y ros-$ROS_DISTRO-cartographer-ros

sudo apt-get install -y python-yaml
sudo apt install -y git
sudo apt-get install ros-$ROS_DISTRO-serial -y
sudo apt install pcl-tools -y

sudo apt install ros-noetic-executive-smach-visualization -y
sudo apt install ros-noetic-smach-viewer -y
sudo apt install -y ros-noetic-gazebo* -y
sudo apt install -y ros-noetic-joint-states* -y
sudo apt install ros-noetic-ackermann* -y
sudo apt install ros-noetic-urdf* -y
