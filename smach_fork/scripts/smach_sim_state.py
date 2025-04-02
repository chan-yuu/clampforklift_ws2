#!/usr/bin/env python3

'''
Author: CYUN
Date: 2024-10-07 08:49:51
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-13 03:24:52
FilePath: /smach_fork/scripts/smach_sim_state.py
Description: version:3.0

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import smach
import smach_ros
import rospy
import time
import csv
import rospkg
import csv
import os
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64,UInt8, Int8


rospack = rospkg.RosPack()
package_path = rospack.get_path('smach_fork')

# 存储的文件
csv_file_path = f'{package_path}/data/state_times.csv'
csv_file_point_path = f'{package_path}/data/current_point.csv'

if not os.path.exists(csv_file_path):
    with open(csv_file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["State Name", "Start Time", "End Time", "Specific Start Time", "Specific End Time", "Elapsed Time"])

if not os.path.exists(csv_file_point_path):
    with open(csv_file_point_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["x", "y", "yaw"])


class SetWidthSim(smach.State):
    def __init__(self, tar_width):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_width = tar_width
        self.pub_left_width = rospy.Publisher('/base_link_to_fork_left_horizontal_controller/command', Float64, queue_size=10)
        self.pub_right_width = rospy.Publisher('/base_link_to_fork_right_horizontal_controller/command', Float64, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.cur_width = 0
        self.start_time = None
        self.previous_changed_width = None
        self.last_check_time = None

    def joint_state_callback(self, msg):
        self.cur_width = msg.position[msg.name.index('base_link_to_fork_left_horizontal')]  # 替换为实际关节名称
        self.changed_width = abs(self.tar_width - self.cur_width)


    def execute(self, userdata):
        global num_work
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        rospy.loginfo_once("\033[32m"+'Executing state: SET_WIDTH_SIM'+"\033[0m")
        
        
        while True:
            count = 0
            if self.start_time is None:
                self.start_time = rospy.Time.now()
            self.pub_left_width.publish(Float64(self.tar_width))
            self.pub_right_width.publish(Float64(-self.tar_width))

            if (rospy.Time.now() - self.start_time).to_sec() > 1:
                current_time = rospy.Time.now()
                if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1: # 3s监控一次
                    current_changed_width = abs(self.tar_width - self.cur_width)
                    # print("current_changed_width: ", current_changed_width)
                    if self.previous_changed_width is not None:
                        difference = abs(current_changed_width - self.previous_changed_width)
                        # print("difference: ", difference)
                        if difference < 10:
                            self.start_time = None
                            self.last_check_time = None
                            self.previous_changed_width = None
                            break
                        self.previous_changed_width = current_changed_width
                    else:
                        self.previous_changed_width = current_changed_width
                    self.last_check_time = current_time

            count += 1
            if count % 100 == 0:
                rospy.loginfo(f" \033[33m cur_height:{(self.cur_width):.3f} \033[0m")
                rospy.loginfo(f" \033[33m tar_height:{(self.tar_width):.3f} \033[0m")

            rospy.Rate(100).sleep()
        rospy.loginfo_once("\033[32m"+"Width set successfully"+"\033[0m")

        end_time = time.time()
        elapsed_time = end_time - start_time
        specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
        state_name = f"{self.__class__.__name__}"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                [state_name, start_time, end_time, specific_start_time, specific_end_time, round(elapsed_time, 3)])

        return 'finish'


class SetHeightSim(smach.State):
    def __init__(self, tar_height):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_height = tar_height
        self.pub_left_vertical = rospy.Publisher('/fork_left_horizontal_to_fork_left_vertical_controller/command', Float64, queue_size=10)
        self.pub_right_vertical = rospy.Publisher('/fork_right_horizontal_to_fork_right_vertical_controller/command', Float64, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.cur_height = 0.0  # 初始化当前高度
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

    def joint_state_callback(self, msg):
        self.cur_height = msg.position[msg.name.index('fork_left_horizontal_to_fork_left_vertical')]  # 替换为实际关节名称
        self.changed_height = abs(self.tar_height - self.cur_height)

    def execute(self, userdata):
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))
        count = 0
        rospy.loginfo_once("\033[32m"+'Executing state: SET_HEIGHT_SIM'+"\033[0m")
        while True:
            if self.start_time is None:
                self.start_time = rospy.Time.now()
            self.pub_left_vertical.publish(Float64(self.tar_height))
            self.pub_right_vertical.publish(Float64(self.tar_height))

            if (rospy.Time.now() - self.start_time).to_sec() > 1:
                current_time = rospy.Time.now()
                if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1.0:
                    current_changed_height = abs(self.tar_height - self.cur_height)
                    if self.previous_changed_height is not None:
                        difference = abs(current_changed_height - self.previous_changed_height)
                        if difference < 10:
                            self.start_time = None
                            self.last_check_time = None
                            self.previous_changed_height = None
                            break
                        self.previous_changed_height = current_changed_height
                    else:
                        self.previous_changed_height = current_changed_height
                    self.last_check_time = current_time

            count += 1
            if count % 100 == 0: 
                rospy.loginfo(f" \033[33m cur_height:{(self.cur_height):.3f} \033[0m")
                rospy.loginfo(f" \033[33m tar_height:{(self.tar_height):.3f} \033[0m")
            
            rospy.Rate(100).sleep()

        rospy.loginfo_once("\033[32m"+"Height set successfully"+"\033[0m")

        end_time = time.time()
        elapsed_time = end_time - start_time
        specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
        state_name = f"{self.__class__.__name__}"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                [state_name, start_time, end_time, specific_start_time, specific_end_time, round(elapsed_time, 3)])

        return 'finish'
