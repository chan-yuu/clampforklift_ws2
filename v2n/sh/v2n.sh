#!/bin/bash
###
 # @Author: CYUN && cyun@tju.enu.cn
 # @Date: 2024-07-24 14:34:01
 # @LastEditors: CYUN && cyun@tju.enu.cn
 # @LastEditTime: 2024-12-22 20:03:21
 # @FilePath: /undefined/home/nvidia/clamp_forklift_ws2/src/v2n/sh/v2n.sh
 # @Description: 启动作业的三个程序（无终端）
 # 
 # Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
### 
# WORKSPACE_PATH=/home/nvidia/10.11_ws
WORKSPACE_PATH="/home/nvidia/clamp_forklift_ws2"

# debug版
# gnome-terminal --tab --title="rviz_ui" -- bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch car_ori_display sim.launch"
# sleep 3s
# gnome-terminal --tab --title="lidar_fusion" -- bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch fusion_pointclouds fusion_pointclouds.launch"
# sleep 1s
# gnome-terminal --tab --title="location" -- bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch perception location_ndt_gps.launch"
# sleep 2s
# gnome-terminal --tab --title="camera" -- bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch camera_dev camera_node.launch"
# sleep 3s
# gnome-terminal --tab --title="fusion_detect" -- bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch lidar_camera_det lidar_camera_det.launch"
# sleep 1s
# gnome-terminal --tab --title="planning" -- bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch hybrid_a_star run_hybrid_a_star.launch"
# sleep 1s
# gnome-terminal --tab --title="control" -- bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch control vehicle_control.launch"
# sleep 1s
gnome-terminal --tab --title="v2n" -- bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch v2n v2n.launch"
# sleep 3s
# gnome-terminal --tab --title="smach" -- bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch smach_fork smach_fork.launch"


# # 无终端版
# # rviz_ui
# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch car_ori_display sim.launch" > /dev/null 2>&1 &
# sleep 3s
# # lidar_fusion
# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch fusion_pointclouds fusion_pointclouds.launch" > /dev/null 2>&1 &
# sleep 1s
# # location
# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch perception location_ndt_gps.launch" > /dev/null 2>&1 &
# sleep 2s
# # camera
# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch camera_dev camera_node.launch" > /dev/null 2>&1 &
# sleep 3s
# # fusion_detect
# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch lidar_camera_det lidar_camera_det.launch" > /dev/null 2>&1 &
# sleep 1s
# planning
# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch hybrid_a_star run_hybrid_a_star.launch" > /dev/null 2>&1 &
# sleep 1s
# # control
# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch control vehicle_control.launch" > /dev/null 2>&1 &
# sleep 1s
# # v2n
# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch v2n v2n.launch" > /dev/null 2>&1 &
# sleep 3s
# smach
# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch smach_fork smach_fork.launch" > /dev/null 2>&1 &

# nohup bash -c "source $WORKSPACE_PATH/devel/setup.bash;roslaunch v2n v2n.launch" > /dev/null 2>&1 &