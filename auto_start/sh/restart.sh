#!/bin/bash
###
 # @Author: CYUN && cyun@tju.enu.cn
 # @Date: 2024-07-24 14:34:01
 # @LastEditors: CYUN && cyun@tju.enu.cn
 # @LastEditTime: 2024-09-14 17:41:15
 # @FilePath: /undefined/home/nvidia/clamp_forklift_ws/src/auto_start/sh/restart.sh
 # @Description: start the start.sh script
 # 
 # Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
### 
WORKSPACE_PATH="/home/cyun/learn_ws/clamp_forklift_ws"
# nohup bash -c 'source /home/cyun/forklift_sim_ws/devel/setup.bash && rosrun forklift_gazebo 1.py >/dev/null 2>&1' &
gnome-terminal --tab --title="restart" -- bash -c "source $WORKSPACE_PATH/src/auto_start/sh/start.sh"
