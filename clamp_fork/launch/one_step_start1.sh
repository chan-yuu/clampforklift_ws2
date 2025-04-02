#!/bin/bash
#description: 开机一键启动
##修改.bashrc让其可以非终端启动

gnome-terminal --tab --title="gps_pub" -- bash -c "source /home/kemove/forklift_sim_ws/devel/setup.bash;roslaunch ndt_localizer ndt_localizer.launch; exec bash"

sleep 2s
gnome-terminal --tab --title="control" -- bash -c "source /home/kemove/forklift_sim_ws/devel/setup.bash;roslaunch control_pkg control.launch; exec bash"
sleep 1s
# gnome-terminal --tab --title="hybrid_a_star" -- bash -c "source ~/.bashrc;roslaunch clamp_fork clamp_control.launch; exec bash"
# gnome-terminal --tab --title="hybrid_a_star" -- bash -c "source ~/.bashrc;roslaunch hybrid_a_star run_hybrid_a_star.launch; exec bash"
# gnome-terminal --tab --title="hybrid_a_star" -- bash -c "source ~/.bashrc;roslaunch hybrid_a_star run_hybrid_a_star.launch; exec bash"



#roscore
# gnome-terminal --tab --title="roscore" -- bash -c "source ~/las/devel/setup.bash;export ROS_HOSTNAME=localhost;export ROS_MASTER_URI=http://localhost:11311;roscore; exec bash"

# sleep 1s
# # 开lidar
# gnome-terminal --tab --title="lidar_ex" -- bash -c "source ~/las/devel/setup.bash;rosrun lidar lidar; exec bash"

# # 开gps
# sleep 1s
# ##NOTE 
# #我没有加吗？？
# gnome-terminal --tab --title="gps_imu_pub" -- bash -c "source ~/las/devel/setup.bash;sudo chmod 777 /dev/ttyUSB0;rosrun perception gps_imu_pub.py; exec bash"

# # # camera
# sleep 1s
# gnome-terminal --tab --title="camera" -- bash -c "/home/nvidia/Downloads/envs/YOLO/bin/python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py; exec bash"

# # 自检
# sleep 1s
# gnome-terminal --tab --title="self_check" -- bash -c "source ~/las/devel/setup.bash;rosrun faultdiagnosis self_check.py; exec bash"

# #故障诊断
# #NOTE
# gnome-terminal --tab --title="fault_diagnosis" -- bash -c "source ~/las/devel/setup.bash;rosrun faultdiagnosis fault_diagnosis.py; exec bash"

# # lidar-sdk
# sleep 1s
# gnome-terminal --tab --title="lidar_sdk" -- bash -c "cd /home/nvidia/RoboSense/release_orin/release/build/demo;source ~/.bashrc;./rs_sdk_demo; exec bash"

# sleep 1s
# # rviz
# ##路径显示添加上去
# gnome-terminal --tab --title="RViz" -- bash -c "source ~/las/devel/setup.bash; rosrun rviz rviz -d /home/nvidia/RoboSense/release_orin/release/config/rviz/perception.rviz; exec bash"
# gnome-terminal --tab --title="RViz" -- bash -c "source ~/las/devel/setup.bash; rosrun rviz rviz -d /home/nvidia/las/src/planning/config/rviz15.rviz; exec bash"


# sleep 1s
# gnome-terminal --tab --title="Hmi" -- bash -c "source ~/las/devel/setup.bash;rosrun hmi hmi_node;exec bash"


# ##终端不显示版：
# sleep 1s
# nohup bash -c "source ~/.bashrc;export ROS_HOSTNAME=localhost;export ROS_MASTER_URI=http://localhost:11311;roscore" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun lidar lidar" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun perception gps_imu_pub.py" &
# sleep 1s
# nohup bash -c "/home/nvidia/Downloads/envs/YOLO/bin/python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun faultdiagnosis self_check.py" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun faultdiagnosis fault_diagnosis.py" &
# sleep 1s
# nohup bash -c "cd /home/nvidia/RoboSense/release_orin/release/build/demo;source ~/.bashrc;./rs_sdk_demo" &
# sleep 1s
# nohup bash -c "source ~/.bashrc; rosrun rviz rviz -d /home/nvidia/RoboSense/release_orin/release/config/rviz/perception.rviz" &
# sleep 1s
# nohup bash -c "source ~/.bashrc; rosrun hmi hmi_node" &


