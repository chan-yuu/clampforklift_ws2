#!/usr/bin/env python
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-07-24 14:34:29
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-09-28 01:28:23
FilePath: /src/auto_start/ui/forklift_start.py
Description: 功能启动UI开发

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import sys
import os
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QGridLayout, QHBoxLayout, QFrame, QProgressBar
from PyQt5.QtCore import Qt, QTimer, QTime
from PyQt5.QtGui import QIcon, QFont, QPixmap
from geometry_msgs.msg import Twist, TwistStamped
import time

# 设置目录
CARLA_PATH = os.environ.get('CARLA_PATH', "/home/cyun/disk1/CARLA_0.9.14") # 修改你的目录
WORKSPACE_PATH = os.environ.get('WORKSPACE_PATH', "/home/cyun/learn_ws/clamp_forklift_ws3") # 修改你的目录
ICON_PATH = os.path.join(WORKSPACE_PATH, "src/auto_start/png")

def run_command(command):
    os.system(f'terminator -e  "{command}" &')

def run_command_restart(command):
    os.system(f'gnome-terminal -e  "{command}" &')
def stop_command(command_name):
    os.system(f'pkill -f "{command_name}"')

def start_top(button):
    run_command("bash -c ' top;'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_top(button):
    stop_command("top")
    button.setStyleSheet('')

def start_urdf(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch forklift_description real_clamp.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_urdf(button):
    stop_command("real_clamp.launch")
    button.setStyleSheet('')

def start_rviz(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch fusion_pointclouds fusion_pointclouds.launch;exec bash'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_rviz(button):
    stop_command("fusion_pointclouds.launch")
    button.setStyleSheet('')

def start_rviz_ui(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch car_ori_display sim.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_rviz_ui(button):
    stop_command("sim.launch")
    button.setStyleSheet('')

def start_forklift_grey_world(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch auto_start record.launch;'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_forklift_grey_world(button):
    stop_command("record.launch")
    button.setStyleSheet('')

def start_lidar_full_gazebo(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch perception odom.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_lidar_full_gazebo(button):
    stop_command("odom.launch")
    button.setStyleSheet('')

def start_lego_loam(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;rosrun auto_start pub_ui.py'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_lego_loam(button):
    stop_command("pub_ui.py")
    button.setStyleSheet('')

def start_lio_sam(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch control pid_controller.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_lio_sam(button):
    stop_command("pid_controller.launch")
    button.setStyleSheet('')

def start_forklift_gmapping(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch control contron_fsm.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_forklift_gmapping(button):
    stop_command("contron_fsm.launch")
    button.setStyleSheet('')

def start_yolov5ros(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch control can_pub.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_yolov5ros(button):
    stop_command("can_pub.launch")
    button.setStyleSheet('')

def start_forklift_nav(button):
    run_command(f"bash -c 'source /home/nvidia/.bashrc;source {WORKSPACE_PATH}/devel/setup.bash; roslaunch mpc_local_planner mpc.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_forklift_nav(button):
    stop_command("mpc.launch")
    button.setStyleSheet('')

def start_forklift_mpc(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch ndt_localizer ndt_localizer.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_forklift_mpc(button):
    stop_command("ndt_localizer.launch")
    button.setStyleSheet('')

def start_keyboard_control(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch control can_feedback.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_keyboard_control(button):
    stop_command("can_feedback.launch")
    button.setStyleSheet('')

def start_det_3d(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch perception gps_in_odom.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_det_3d(button):
    stop_command("gps.launch")
    button.setStyleSheet('')

def start_uav_sim(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;rosrun plotjuggler plotjuggler'")
    # time.sleep(6)
    # run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch uav_simulation uav_simulation.launch'")
    # time.sleep(1)
    # run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch uav_simulation referee_system.launch'")
    # time.sleep(1)
    # run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;rosrun uav_simulation command_process.py'")
    # time.sleep(1)
    # run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;rosrun uav_simulation keyboard_control.py'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_uav_sim(button):
    stop_command("plotjuggler")
    # stop_command("uav_simulation.launch")
    # stop_command("referee_system.launch")
    # stop_command("command_process.py")
    # stop_command("keyboard_control.py")
    button.setStyleSheet('')

def start_ollama(button):
    run_command("bash -c 'rqt;'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_ollama(button):
    stop_command("rqt")
    button.setStyleSheet('')

def start_label(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; roslaunch lidar_camera_det lidar_camera_det.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_label(button):
    stop_command("lidar_camera_det.launch")
    button.setStyleSheet('')

def start_rl_td3(button):
    run_command(f"bash -c 'cd {WORKSPACE_PATH};catkin_make -DCATKIN_WHITELIST_PACKAGES=car_interfaces'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_rl_td3(button):
    stop_command("train_velodyne_td3.launch")
    button.setStyleSheet('')

def start_label_cloud(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch hybrid_a_star run_hybrid_a_star.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_label_cloud(button):
    stop_command("run_hybrid_a_star.launch")
    button.setStyleSheet('')

def start_joy_control(button):
    run_command(f"bash -c 'cd {WORKSPACE_PATH};catkin_make -DCATKIN_WHITELIST_PACKAGES= ;  '")
    time.sleep(1)
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch joy_control joy_control.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_joy_control(button):
    stop_command("jstest")
    stop_command("pub_joy.launch")
    button.setStyleSheet('')

def start_forklift_sim(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch forklift_nav forklift_nav_runner.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_forklift_sim(button):
    stop_command("forklift_nav_runner.launch")
    button.setStyleSheet('')

def start_forklift_nav_2(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch local_plan local_plan_with_map.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_forklift_nav_2(button):
    stop_command("local_plan_with_map.launch")
    button.setStyleSheet('')

def start_forklift_mpc_2(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch smach_fork smach_fork.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_forklift_mpc_2(button):
    stop_command("smach_fork.launch")
    button.setStyleSheet('')

def start_keyboard_control_2(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;rosrun joy_control keyboard_control.py'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_keyboard_control_2(button):
    stop_command("keyboard_control.py")
    button.setStyleSheet('')

def start_cotton_forklift(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch forklift_gazebo clamp_forklift_sim.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_cotton_forklift(button):
    stop_command("clamp_forklift_sim.launch")
    button.setStyleSheet('')

def start_cotton_joy_control(button):
    run_command(f"bash -c 'jstest /dev/input/js0;'")
    time.sleep(1)
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch joy_control cotton_joy.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')

def stop_cotton_joy_control(button):
    stop_command("jstest")
    stop_command("cotton_joy.launch")
    button.setStyleSheet('')

def start_cotton_keyboard_control(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;rosrun joy_control cotton_keyboard_control.py'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_cotton_keyboard_control(button):
    stop_command("cotton_keyboard_control.py")
    button.setStyleSheet('')

def start_cotton_sim2(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch forklift_gazebo spawn_fork.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_cotton_sim2(button):
    stop_command("spawn_fork.launch")
    button.setStyleSheet('')


def start_cotton_sim3(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch clamp_fork spawn_fork.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_cotton_sim3(button):
    stop_command("spawn_fork.launch")
    button.setStyleSheet('')

def start_cotton_loc(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch clamp_fork ndt_localizer.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_cotton_loc(button):
    stop_command("ndt_localizer.launch")
    button.setStyleSheet('')

def start_cotton_con(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch control_pkg control.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_cotton_con(button):
    stop_command("control.launch")
    button.setStyleSheet('')
def start_cotton_hybrid(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch hybrid_a_star run_hybrid_a_star.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_cotton_hybrid(button):
    stop_command("run_hybrid_a_star.launch")
    button.setStyleSheet('')

def start_cotton_smach(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch smach_fork smach_service.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_cotton_smach(button):
    stop_command("smach_service.launch")
    button.setStyleSheet('')

def start_cotton_smach_auto(button):
    run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch smach_fork smach_service_auto.launch'")
    button.setStyleSheet('background-color: #5e35b1; color: white;')
def stop_cotton_smach_auto(button):
    stop_command("smach_service_auto.launch")
    button.setStyleSheet('')

class RuntimeManager(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        rospy.init_node('runtime_manager', anonymous=True)
        self.cmd_vel_subscriber = rospy.Subscriber('/twist', TwistStamped, self.update_velocity)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_received_time = QTime.currentTime()  # Record the time when a message is received
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_velocity_display)
        self.timer.start(100)
    def initUI(self):
        self.setWindowTitle('Runtime Manager')
        self.setWindowIcon(QIcon(f'{ICON_PATH}/icon.png'))

        # Styling
        button_style = 'font-size: 12pt; padding: 10px; min-width: 60px; min-height: 20px;'
        active_button_style = 'background-color: #5e35b1; color: white; font-size: 12pt; padding: 10px; min-width: 80px; min-height: 20px;'

        # Layouts
        main_layout = QVBoxLayout()
        grid_layout = QGridLayout()

        commands = [
            ("Top", start_top, stop_top),
            # ("Start RL-TD3", start_rl_td3, stop_rl_td3),
            # ("Start Joy Control", start_joy_control, stop_joy_control),
            ("Rviz UI", start_rviz_ui, stop_rviz_ui),
            ("Clamp Fork URDF", start_urdf, stop_urdf),
            ("Lidar Fusion", start_rviz, stop_rviz),
            ("L-Camera det", start_label, stop_label),
            ("Path Record", start_forklift_grey_world, stop_forklift_grey_world),
            ("GPS", start_det_3d, stop_det_3d),
            ("Odom", start_lidar_full_gazebo, stop_lidar_full_gazebo),
            ("NDT", start_forklift_mpc, stop_forklift_mpc),
            ("Path", start_lego_loam, stop_lego_loam),
            ("Global planning-A*", start_label_cloud, stop_label_cloud),
            ("Local planning-A*", start_forklift_nav_2, stop_forklift_nav_2),
            ("Control FSM", start_forklift_gmapping, stop_forklift_gmapping),
            ("Can Pub", start_yolov5ros, stop_yolov5ros),
            ("Smach", start_forklift_mpc_2, stop_forklift_mpc_2),
            ("Can Feedback", start_keyboard_control, stop_keyboard_control),
            ("MPC", start_forklift_nav, stop_forklift_nav),
            ("Poltjuggle", start_uav_sim, stop_uav_sim),
            ("RQT", start_ollama, stop_ollama)
            # ("Start Joy Control", start_joy_control, stop_joy_control),
            # ("Forklift Nav 2.0", start_forklift_nav_2, stop_forklift_nav_2),
            # ("Forklift MPC 2.0", start_forklift_mpc_2, stop_forklift_mpc_2),
            # ("Forklift Keyboard 2.0", start_keyboard_control_2, stop_keyboard_control_2),

            # ("Cotton Forklift Sim", start_cotton_forklift, stop_cotton_forklift),
            # ("Cotton Forklift Joy", start_cotton_joy_control, stop_cotton_joy_control),
            # ("Cotton Forklift Keyboard", start_cotton_keyboard_control, stop_cotton_keyboard_control),
            
            # ("Cotton Forklift 2.0 SIM", start_cotton_sim2, stop_cotton_sim2),

            # ("Cotton Forklift 3.0 SIM", start_cotton_sim3, stop_cotton_sim3),
            # ("Cotton Forklift Location", start_cotton_loc, stop_cotton_loc),
            # ("Cotton Forklift Control", start_cotton_con, stop_cotton_con),
            # ("Cotton Forklift Hybrid", start_cotton_hybrid, stop_cotton_hybrid),
            # ("Cotton Forklift Smach", start_cotton_smach, stop_cotton_smach),
            # ("Cotton Forklift Smach Auto", start_cotton_smach_auto, stop_cotton_smach_auto)
        ]

        total_commands = len(commands)
        left_count = (total_commands + 1) // 2
        right_count = total_commands // 2

        # Fill left column
        for index in range(left_count):
            title, start_command, stop_command = commands[index]
            label = QLabel(title)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet('font-size: 14pt;')

            start_button = QPushButton('Start')
            start_button.setStyleSheet(button_style)
            start_button.setFixedSize(150, 60)
            start_button.clicked.connect(lambda _, b=start_button, cmd=start_command: cmd(b))

            stop_button = QPushButton('Stop')
            stop_button.setStyleSheet(button_style)
            stop_button.setFixedSize(150, 60)
            stop_button.clicked.connect(lambda _, b=start_button, cmd=stop_command: cmd(b))

            grid_layout.addWidget(label, index, 0)
            grid_layout.addWidget(start_button, index, 1)
            grid_layout.addWidget(stop_button, index, 2)

        # Fill right column
        for index in range(right_count):
            title, start_command, stop_command = commands[left_count + index]
            label = QLabel(title)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet('font-size: 14pt;')

            start_button = QPushButton('Start')
            start_button.setStyleSheet(button_style)
            start_button.setFixedSize(150, 60)
            start_button.clicked.connect(lambda _, b=start_button, cmd=start_command: cmd(b))

            stop_button = QPushButton('Stop')
            stop_button.setStyleSheet(button_style)
            stop_button.setFixedSize(150, 60)
            stop_button.clicked.connect(lambda _, b=start_button, cmd=stop_command: cmd(b))

            grid_layout.addWidget(label, index, 3)
            grid_layout.addWidget(start_button, index, 4)
            grid_layout.addWidget(stop_button, index, 5)


        restart_button_style = 'background-color: #08f311; color: black; font-size: 12pt; padding: 10px; min-width: 80px; min-height: 20px;'

        # 添加编译功能：
        catkin_button_style = 'background-color: #08f311; color: black; font-size: 12pt; padding: 10px; min-width: 80px; min-height: 20px;'
        shutdown_button_style = 'background-color: #C40000; color: black; font-size: 12pt; padding: 10px; min-width: 80px; min-height: 20px;'
        horizontal_layout = QHBoxLayout()
        buttons = [
            ("Restart All",restart_button_style, self.restart_all),
            ("Catkin-Make",catkin_button_style, self.catkin_make),
            ("Shutdown",shutdown_button_style, self.shutdown)
        ]

        for title, style, command in buttons:
            button = QPushButton(title)
            button.setStyleSheet(style)
            button.setFixedSize(150, 60)
            button.clicked.connect(command)
            horizontal_layout.addWidget(button)

        main_layout.addLayout(horizontal_layout)

        # box Display
        velocity_layout = QHBoxLayout()

        # linear velocity display
        linear_frame = QFrame()
        linear_frame.setFrameShape(QFrame.StyledPanel)
        linear_frame.setStyleSheet('background-color: #e1f5fe; border: 2px solid #0288d1; border-radius: 10px; padding: 10px;')
        linear_layout = QVBoxLayout()

        self.linear_velocity_bar = QProgressBar()
        self.linear_velocity_bar.setOrientation(Qt.Vertical)
        self.linear_velocity_bar.setRange(0, 100)
        self.linear_velocity_bar.setValue(0)
        self.linear_velocity_bar.setTextVisible(False)
        self.linear_velocity_bar.setStyleSheet('QProgressBar::chunk {background-color: #0288d1;}')
        linear_layout.addWidget(self.linear_velocity_bar, alignment=Qt.AlignCenter)

        self.linear_velocity_label = QLabel("0.0 m/s")
        self.linear_velocity_label.setAlignment(Qt.AlignCenter)
        self.linear_velocity_label.setFont(QFont('Arial', 14))
        linear_layout.addWidget(self.linear_velocity_label, alignment=Qt.AlignCenter)

        linear_frame.setLayout(linear_layout)
        velocity_layout.addWidget(linear_frame)

        # angular velocity display
        angular_frame = QFrame()
        angular_frame.setFrameShape(QFrame.StyledPanel)
        angular_frame.setStyleSheet('background-color: #e1f5fe; border: 2px solid #d32f2f; border-radius: 10px; padding: 10px;')
        angular_layout = QVBoxLayout()

        self.angular_velocity_bar = QProgressBar()
        self.angular_velocity_bar.setOrientation(Qt.Vertical)
        self.angular_velocity_bar.setRange(0, 100)
        self.angular_velocity_bar.setValue(0)
        self.angular_velocity_bar.setTextVisible(False)
        self.angular_velocity_bar.setStyleSheet('QProgressBar::chunk {background-color: #d32f2f;}')
        angular_layout.addWidget(self.angular_velocity_bar, alignment=Qt.AlignCenter)

        self.angular_velocity_label = QLabel("0.0 rad/s")
        self.angular_velocity_label.setAlignment(Qt.AlignCenter)
        self.angular_velocity_label.setFont(QFont('Arial', 14))
        angular_layout.addWidget(self.angular_velocity_label, alignment=Qt.AlignCenter)

        angular_frame.setLayout(angular_layout)
        velocity_layout.addWidget(angular_frame)

        # height display
        height_frame = QFrame()
        height_frame.setFrameShape(QFrame.StyledPanel)
        height_frame.setStyleSheet('background-color: #e1f5fe; border: 2px solid #0288d1; border-radius: 10px; padding: 10px;')
        height_layout = QVBoxLayout()

        self.height_velocity_bar = QProgressBar()
        self.height_velocity_bar.setOrientation(Qt.Vertical)
        self.height_velocity_bar.setRange(0, 100)
        self.height_velocity_bar.setValue(0)
        self.height_velocity_bar.setTextVisible(False)
        self.height_velocity_bar.setStyleSheet('QProgressBar::chunk {background-color: #0288d1;}')
        height_layout.addWidget(self.height_velocity_bar, alignment=Qt.AlignCenter)

        self.height_velocity_label = QLabel("0.0 m")
        self.height_velocity_label.setAlignment(Qt.AlignCenter)
        self.height_velocity_label.setFont(QFont('Arial', 14))
        height_layout.addWidget(self.height_velocity_label, alignment=Qt.AlignCenter)

        height_frame.setLayout(height_layout)
        velocity_layout.addWidget(height_frame)

        main_layout.addLayout(velocity_layout)
        main_layout.addLayout(grid_layout)

        # Add author information
        author_label = QLabel('Contributed by CYUN')
        author_label.setAlignment(Qt.AlignCenter)
        author_label.setStyleSheet('font-size: 12pt; color: black;')
        main_layout.addWidget(author_label)
        self.setLayout(main_layout)

    def restart_all(self):
        # 停止所有命令
        stop_command("roscore")
        stop_command("forklift_gazebo.launch")
        stop_command("forklift_grey_world.launch")
        stop_command("lidar_full_gazebo.launch")
        stop_command("lio_sam run.launch")
        stop_command("lego_loam run.launch")
        stop_command("forklift_gmapping.launch")
        stop_command("yolov5ros start.launch")
        stop_command("forklift_nav.launch")

        stop_command("uav_simulation.launch")
        stop_command("env_simulation.launch")
        stop_command("referee_system.launch")
        stop_command("command_process.py")
        stop_command("keyboard_control.py")

        # 启动指定的两个命令
        run_command_restart(f"bash -c '{WORKSPACE_PATH}/src/auto_start/sh/restart.sh'")
        
        # 退出当前应用程序
        QApplication.quit()

    def catkin_make(self):
        # 进入工作空间编译
        run_command(f"bash -c 'source /home/nvidia/.bashrc;cd {WORKSPACE_PATH};catkin_make; exec bash;'")

    def shutdown(self):
        # 关机
        # run_command(f"bash -c 'cd {WORKSPACE_PATH};source {WORKSPACE_PATH}/devel/setup.bash;catkin_make; exec bash;'")
        stop_command("roscore")
        stop_command("forklift_start.py")
        QApplication.quit()
    def update_velocity(self, msg):
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z
        self.last_received_time = QTime.currentTime()  # Update the time when a message is received

    def refresh_velocity_display(self):

        self.linear_velocity_label.setText(f"{self.linear_velocity:.2f} m/s")
        self.angular_velocity_label.setText(f"{self.angular_velocity:.2f} rad/s")
        self.angular_velocity_label.setText(f"{self.angular_velocity:.2f} rad/s")

        # self.linear_velocity = abs(self.linear_velocity)
        # self.angular_velocity = abs(self.angular_velocity)

        self.linear_velocity = (self.linear_velocity)
        self.angular_velocity = (self.angular_velocity)

        # Check if the time since the last message is greater than 500ms
        if self.last_received_time.msecsTo(QTime.currentTime()) > 500:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
        
        # Assume max linear velocity is 4 m/s and max angular velocity is 2 rad/s for scaling
        max_linear_velocity = 4.0
        max_angular_velocity = 2.0
        
        self.linear_velocity_bar.setValue(int((self.linear_velocity / max_linear_velocity) * 100))
        self.angular_velocity_bar.setValue(int((self.angular_velocity / max_angular_velocity) * 100))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RuntimeManager()
    ex.show()
    sys.exit(app.exec_())