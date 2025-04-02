#!/bin/bash
###
 # @Author: CYUN && cyun@tju.enu.cn
 # @Date: 2024-07-24 14:34:01
 # @LastEditors: CYUN && cyun@tju.enu.cn
 # @LastEditTime: 2024-12-18 22:18:36
 # @FilePath: /test_plan_ws/home/cyun/forklift_sim_ws3/src/auto_start/sh/start_sim_forklift.sh
 # @Description: 
 # 
 # Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
### 
# WORKSPACE_PATH="/home/cyun/forklift_sim_ws3"
# nohup bash -c 'source /opt/ros/noetic/setup.bash;source $WORKSPACE_PATH/devel/setup.bash; roscore ' &
# nohup bash -c 'source /opt/ros/noetic/setup.bash;source $WORKSPACE_PATH/devel/setup.bash && rosrun forklift_gazebo 1.py ' &

WORKSPACE_PATH="/home/cyun/forklift_sim_ws3"

# 使用nohup运行roscore命令，将输出重定向到 /dev/null（即丢弃输出，实现静默运行）
nohup bash -c "source /opt/ros/noetic/setup.bash;source $WORKSPACE_PATH/devel/setup.bash; roslaunch auto_start ui.launch" > /dev/null 2>&1 &

# 使用nohup运行rosrun命令，同样将输出重定向到 /dev/null
# nohup bash -c "source /opt/ros/noetic/setup.bash;source $WORKSPACE_PATH/devel/setup.bash;rosrun forklift_gazebo 1.py" > /dev/null 2>&1 &

# gnome-terminal --tab --title="keyboard_auto_all" -- bash -c "source /opt/ros/noetic/setup.bash;source $WORKSPACE_PATH/devel/setup.bash; roscore"
# gnome-terminal --tab --title="start" -- bash -c "source /opt/ros/noetic/setup.bash;source $WORKSPACE_PATH/devel/setup.bash && rosrun forklift_gazebo 1.py"



# terminator -e "bash -c 'source ~/.bashrc; watch -n 1 nvidia-smi;'" & sleep 1 && wmctrl -r :ACTIVE: -e 0,600,600,850,500
# gnome-terminal --tab --title="keyboard_auto_all" -- bash -c "source /home/cyun/forklift_sim_ws/devel/setup.bash; roscore; exec bash;"
# sleep 1s
# # 启动第一个终端，运行keyboard_auto_all.py脚本
# gnome-terminal --tab --title="keyboard_auto_all" -- bash -c "source /home/cyun/forklift_sim_ws/devel/setup.bash; rosrun forklift_gazebo keyboard_auto_all.py; exec bash;"

# sleep 1s
# # 启动第二个终端，并根据用户选择运行forklift_gazebo.launch
# gnome-terminal --tab --title="forklift_gazebo" -- bash -c "
#   read -p 'Run forklift_gazebo simulation? (0-No, 1-Yes 2-Use_Network, 3-LidarFull): ' choice;
#   if [ \$choice -eq 1 ]; then
#     source /home/cyun/forklift_sim_ws/devel/setup.bash;
#     roslaunch forklift_gazebo forklift_gazebo.launch;
#   elif [ \$choice -eq 2 ]; then
#     source /home/cyun/forklift_sim_ws/devel/setup.bash;
#     roslaunch forklift_gazebo forklift_grey_world.launch;
#   elif [ \$choice -eq 3 ]; then
#     source /home/cyun/forklift_sim_ws/devel/setup.bash;
#     roslaunch forklift_gazebo lidar_full_gazebo.launch;
#   fi;
#   exec bash;"

# sleep 2s
# # 启动第三个终端，并根据用户选择运行run.launch
# gnome-terminal --tab --title="lio_sam" -- bash -c "
#   read -p 'Run lio-sam mapping? (0-No, 1-Yes): ' choice;
#   if [ \$choice -eq 1 ]; then
#     source /home/cyun/forklift_sim_ws/devel/setup.bash;
#     roslaunch lio_sam run.launch;
#   fi;
#   exec bash;"


# sleep 2s
# # 启动第三个终端，并根据用户选择运行run.launch
# gnome-terminal --tab --title="gmapping" -- bash -c "
#   read -p 'Run lio-sam mapping? (0-No, 1-Yes): ' choice;
#   if [ \$choice -eq 1 ]; then
#     source /home/cyun/forklift_sim_ws/devel/setup.bash;
#     roslaunch forklift_gmapping forklift_gmapping.launch;
#   fi;
#   exec bash;"

# sleep 2s
# # yolo识别：
# gnome-terminal --tab --title="yolo_det" -- bash -c "
#   read -p 'Run yolo-detect ? (0-No, 1-Yes): ' choice;
#   if [ \$choice -eq 1 ]; then
#     source ~/.bashrc;
#     conda activate yolo;
#     source /home/cyun/forklift_sim_ws/devel/setup.bash;
#     roslaunch yolov5ros start.launch;
#   fi;
#   exec bash;"


# while true; do
#   sleep 1
# done
# # 启动其他脚本
# # gnome-terminal --tab --title="additional_script1" -- bash -c "source devel/setup.bash; rosrun forklift_gazebo command_process.py; exec bash;"
# # gnome-terminal --tab --title="additional_script3" -- bash -c "source devel/setup.bash; rosrun forklift_gazebo keyboard_manual.py; exec bash;"