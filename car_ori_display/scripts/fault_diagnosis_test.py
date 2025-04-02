#!/usr/bin/env python3

'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-24 11:33:40
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-09 20:20:26
FilePath: /undefined/home/cyun/forklift_sim_ws3/src/car_ori_display/scripts/fault_diagnosis_test.py
Description: rviz界面故障诊断

Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
'''

import rospy
import random
import requests
import can
from car_interfaces.msg import FaultDiagnosisInterface  # 替换为你的实际包名
import socket

def check_internet_connection(host="www.baidu.com", port=80, timeout=1):
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            return True
    except (socket.timeout, socket.error):
        return False

def check_can_state():
    try:
        bus = can.interface.Bus(channel='can1', bustype='socketcan')
        msg = bus.recv(timeout=5.0)  # 5秒超时
        if msg and msg.arbitration_id == 0x220: # 待修改
            return True
        return False
    except (OSError, can.CanError):
        return False
    
def check_gps_can_state():
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        msg = bus.recv(timeout=5.0)  # 5秒超时
        if msg and msg.arbitration_id == 0x220: # 待修改
            return True
        return False
    except (OSError, can.CanError):
        return False

def fault_diagnosis_publisher():
    pub = rospy.Publisher('fault_diagnosis_data', FaultDiagnosisInterface, queue_size=10)
    rospy.init_node('fault_diagnosis_test', anonymous=True)
    rate = rospy.Rate(10)  # 1 Hz

    while not rospy.is_shutdown():
        msg = FaultDiagnosisInterface()
        # msg.can_state = 1 if check_can_state() else 0  # 检查CAN状态
        # msg.gps_can_state = 1 if check_gps_can_state() else 0  # 检查CAN状态
        # msg.internet_state = 1 if check_internet_connection() else 0  # 检查互联网连接

        # msg.lidar_state = random.randint(0, 1)  # 随机生成0或1
        # msg.gps_system_state = random.randint(0, 1)
        # msg.camera_state = random.randint(0, 1)

        msg.can_state = 1 
        msg.gps_can_state = 1 
        msg.internet_state = 1 
        msg.lidar_state = 1
        msg.gps_system_state = 1
        msg.camera_state = 1

        rospy.loginfo_once("Publishing fault diagnosis data: %s", msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        fault_diagnosis_publisher()
    except rospy.ROSInterruptException:
        pass
