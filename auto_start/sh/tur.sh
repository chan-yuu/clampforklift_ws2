#!/bin/bash
###
 # @Author: CYUN && cyun@tju.enu.cn
 # @Date: 2024-07-24 14:34:01
 # @LastEditors: CYUN && cyun@tju.enu.cn
 # @LastEditTime: 2024-12-19 16:53:03
 # @FilePath: /undefined/home/cyun/forklift_sim_ws3/src/auto_start/sh/tur.sh
 # @Description: 
 # 
 # Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
### 
# nohup bash -c "source /opt/ros/noetic/setup.bash;roscore" > /dev/null 2>&1 &
# nohup bash -c "source /opt/ros/noetic/setup.bash;rosrun turtlesim turtlesim_node" > /dev/null 2>&1 &

gnome-terminal --tab --title="restart" -- bash -c "source /opt/ros/noetic/setup.bash;roscore"
gnome-terminal --tab --title="1" -- bash -c "source /opt/ros/noetic/setup.bash;rosrun turtlesim turtlesim_node"
