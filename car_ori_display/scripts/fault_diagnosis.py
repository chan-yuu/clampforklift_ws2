#!/usr/bin/env python3
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-16 18:21:37
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-11 02:19:41
FilePath: /src/car_ori_display/scripts/fault_diagnosis.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''

import rospy
import random
import requests
import can
from car_interfaces.msg import FaultDiagnosisInterface,GpsImuInterface  # 替换为你的实际包名
from sensor_msgs.msg import PointCloud2, Image
import socket
import time
import threading
import rospy
import tf2_ros

import rospy
from std_msgs.msg import Bool


lidar_state = 0
camera_state = 0
gps_state = 0
v2n_state = 0

can_state = 0
gps_can_state = 0
internet_state = 0
location_state = 0
stop_threads = False

def lidar_state_callback(msg):
    global lidar_state
    lidar_state = 1


def v2n_callback(msg):
    global v2n_state
    v2n_state = 1



def v2n_pub_callback(msg):
    # global has_received
    # has_received = True
    print("Received v2n_sub_connect")

def check_location():
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        return True
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return False
    
def camera_state_callback(msg):
    global camera_state
    camera_state = 1
    
def gps_state_callback(msg):
    global gps_state
    gps_state = 1

# def check_internet_connection():
#     try:
#         response = requests.get("https://www.baidu.com", timeout=10)
#         return response.status_code == 200
#     except requests.RequestException:
#         return False

# 设置全局默认超时时间（可选）
socket.setdefaulttimeout(0.5)  # 全局超时设置为1秒

def resolve_hostname(host, timeout=1):
    """尝试解析主机名并限制时间"""
    class ResolveThread(threading.Thread):
        def __init__(self, host):
            super().__init__()
            self.host = host
            self.result = None
            self.error = None

        def run(self):
            try:
                self.result = socket.gethostbyname(self.host)
            except Exception as e:
                self.error = e

    resolve_thread = ResolveThread(host)
    resolve_thread.start()
    resolve_thread.join(timeout)

    if resolve_thread.is_alive():
        # 线程仍在运行，说明超时了
        return False, "Timeout resolving hostname"
    elif resolve_thread.error:
        # 线程执行过程中发生了错误
        return False, str(resolve_thread.error)
    else:
        return True, resolve_thread.result

def check_internet_connection(host="www.baidu.com", port=80, timeout=1):
    try:
        # 尝试解析主机名，确保 DNS 正常工作
        success, result = resolve_hostname(host, timeout)
        if not success:
            print(f"Failed to resolve hostname: {result}")
            return False
        
        # 尝试建立 TCP 连接，确保可以到达目标服务器
        with socket.create_connection((host, port), timeout=timeout) as sock:
            # print(f"Successfully connected to {host}:{port}")
            rospy.loginfo_once(f"Successfully connected to {host}:{port}")

        # 尝试发送 HTTP 请求，确保可以通过 HTTP 获取内容
        response = requests.get(f"http://{host}", timeout=timeout)
        if response.status_code == 200:
            # print(f"Successfully received response from {host}")
            return True
        else:
            print(f"Received unexpected status code: {response.status_code}")
            return False

    except (socket.timeout, socket.error):
        print("Failed to connect to host")
        return False
    except requests.RequestException as e:
        print(f"Failed to get response from host: {e}")
        return False
    
def check_can_state():
    try:
        bus = can.interface.Bus(channel='can0', interface='socketcan')
        msg = bus.recv(timeout=1.0)
        if msg is not None:
            return True
        return False
    except (OSError, can.CanError):
        return False

def check_gps_can_state():
    try:
        bus = can.interface.Bus(channel='can1', interface='socketcan')
        msg = bus.recv(timeout=1.0)  # 5秒超时
        if msg: # 待修改
            return True
        return False
    except (OSError, can.CanError):
        return False



def check_gps_can_state():
    try:
        bus = can.interface.Bus(channel='can1', interface='socketcan')
        msg = bus.recv(timeout=1.0)
        if msg:
            return True
        return False
    except (OSError, can.CanError):
        return False


def update_can_state():
    global can_state, stop_threads
    while not stop_threads:
        can_state = 1 if check_can_state() else 0
        time.sleep(1)


def update_gps_can_state():
    global gps_can_state, stop_threads
    while not stop_threads:
        gps_can_state = 1 if check_gps_can_state() else 0
        time.sleep(1)


def update_internet_state():
    global internet_state, stop_threads
    
    while not stop_threads:
        internet_state = 1 if check_internet_connection() else 0
        time.sleep(1)


def update_location_state():
    global location_state, stop_threads
    while not stop_threads:
        location_state = 1 if check_location() else 0
        time.sleep(1)



def fault_diagnosis_publisher():
    global lidar_state, camera_state, gps_state
    global v2n_state, can_state, gps_can_state, location_state, internet_state, stop_threads
    rospy.init_node('fault_diagnosis_test', anonymous=True)
    rate = rospy.Rate(5)
    pub = rospy.Publisher('fault_diagnosis_data', FaultDiagnosisInterface, queue_size=10)

    rospy.Subscriber('/rslidar_points', PointCloud2, lidar_state_callback)
    rospy.Subscriber('/camera/color/image_raw', Image, camera_state_callback)
    rospy.Subscriber('/gps_imu', GpsImuInterface, gps_state_callback)
    rospy.Subscriber('/v2n_sub_connect', Bool, v2n_callback)
    rospy.Subscriber('/v2n_pub_connect', Bool, v2n_pub_callback)


    can_thread = threading.Thread(target=update_can_state)
    gps_can_thread = threading.Thread(target=update_gps_can_state)
    internet_thread = threading.Thread(target=update_internet_state)
    location_thread = threading.Thread(target=update_location_state)

    can_thread.start()
    gps_can_thread.start()
    internet_thread.start()
    location_thread.start()

    try:
        while not rospy.is_shutdown():
            msg = FaultDiagnosisInterface()
            msg.can_state = can_state
            msg.gps_can_state = gps_can_state
            msg.internet_state = internet_state
            msg.location_state = location_state

            msg.v2n_state = v2n_state
            msg.sim_state = 1
            msg.lidar_state = lidar_state
            msg.gps_system_state = gps_state
            msg.camera_state = camera_state

            pub.publish(msg)
            lidar_state = 0
            gps_state = 0
            camera_state = 0
            v2n_state = 0

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        stop_threads = True
        can_thread.join()
        gps_can_thread.join()
        internet_thread.join()
        location_thread.join()


if __name__ == '__main__':
    try:
        fault_diagnosis_publisher()
    except rospy.ROSInterruptException:
        pass
