#!/bin/bash
###
 # @Author: CYUN && cyun@tju.enu.cn
 # @Date: 2024-07-24 14:34:01
 # @LastEditors: CYUN && cyun@tju.enu.cn
 # @LastEditTime: 2024-07-30 04:58:58
 # @FilePath: /CM13_Forklift2024/home/cyun/forklift_sim_ws/start.sh
 # @Description: 
 # 
 # Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
### 
WORKSPACE_PATH="/home/cyun/learn_ws/clamp_forklift_ws"
# nohup bash -c 'source /home/cyun/forklift_sim_ws/devel/setup.bash && rosrun forklift_gazebo 1.py >/dev/null 2>&1' &
gnome-terminal --tab --title="keyboard_auto_all" -- bash -c "source /opt/ros/noetic/setup.bash;source source $WORKSPACE_PATH/devel/setup.bash; roscore"
gnome-terminal --tab --title="start" -- bash -c "source /opt/ros/noetic/setup.bash;source $WORKSPACE_PATH/devel/setup.bash && rosrun auto_start forklift_start.py"