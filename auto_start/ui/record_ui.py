#!/usr/bin/env python
'''
Author: cyun
Date: 2024-09-06 22:32:39
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-09-24 20:20:36
FilePath: /undefined/home/nvidia/clamp_forklift_ws/src/auto_start/ui/record_ui.py
Description: 

Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
'''
import rospy
import os
import json
import signal
import math
import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QFileDialog, QVBoxLayout, QWidget, QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from car_interfaces.msg import GpsImuInterface
from tf.transformations import quaternion_from_euler
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped


class OdomRecorder:
    def __init__(self):
        rospy.init_node('odom_recorder')
        
        self.input_source = rospy.get_param('~input_source', 'odom')
        self.axle_topic = rospy.get_param('~axle_topic', 'base_link')

        if self.input_source == 'odom':
            self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
            rospy.loginfo("Subscribed to /odom topic")
        elif self.input_source == 'gps_imu':
            self.odom_sub = rospy.Subscriber('gps_imu', GpsImuInterface, self.gps_callback)
            rospy.loginfo("Subscribed to gps_imu topic")
        elif self.input_source == 'map_pose':
            self.odom_sub = rospy.Subscriber('map_pose', GpsImuInterface, self.map_pose_callback)
            rospy.loginfo("Subscribed to map_pose topic")
        else:
            rospy.logerr("Unknown input source: {}".format(self.input_source))
            rospy.signal_shutdown("Invalid input source")
            
        self.path = []  
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.1)  
        print("************", self.distance_threshold)
        self.last_pose = None  
        self.is_recording = False  
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        signal.signal(signal.SIGINT, self.signal_handler)

    def odom_callback(self, msg):
        # 从tf中得到新的定位点的位置信息
        if not self.is_recording:
            return

        try:
            # 从 map 到 base_link 获取最新的转换
            transform = self.tf_buffer.lookup_transform("map", self.axle_topic, rospy.Time(0), rospy.Duration(1.0))

            # 使用 transform 的位姿替换 odom 中的位姿
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            # 计算与上一个点的距离
            if self.last_pose is not None:
                distance = self.calculate_distance(self.last_pose, pose)
                if distance < self.distance_threshold:
                    return

            # 记录位姿
            self.path.append({
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z,
                'qx': pose.pose.orientation.x,
                'qy': pose.pose.orientation.y,
                'qz': pose.pose.orientation.z,
                'qw': pose.pose.orientation.w
            })

            self.last_pose = pose
            rospy.loginfo("Recorded Pose: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            ))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: {}".format(e))


    def gps_callback(self, msg):
        print("sub oko")
        if not self.is_recording:
            return

        pose = PoseStamped()
        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = 0#msg.z

        # quaternion = quaternion_from_euler(msg.roll, msg.pitch, msg.yaw)
        quaternion = quaternion_from_euler(0, 0, msg.yaw)

        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        if self.last_pose is not None:
            distance = self.calculate_distance(self.last_pose, pose)
            if distance < self.distance_threshold:
                return

        self.path.append({
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'z': pose.pose.position.z,
            'qx': pose.pose.orientation.x,
            'qy': pose.pose.orientation.y,
            'qz': pose.pose.orientation.z,
            'qw': pose.pose.orientation.w
        })

        self.last_pose = pose
        rospy.loginfo("Recorded Pose: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        ))

    def map_pose_callback(self, msg):
       # 从tf中得到新的定位点的位置信息
        if not self.is_recording:
            return

        try:
            # 从 map 到 base_link 获取最新的转换
            transform = self.tf_buffer.lookup_transform("map", self.axle_topic, rospy.Time(0), rospy.Duration(1.0))

            # 使用 transform 的位姿替换 odom 中的位姿
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            # 计算与上一个点的距离
            if self.last_pose is not None:
                distance = self.calculate_distance(self.last_pose, pose)
                if distance < self.distance_threshold:
                    return

            # 记录位姿
            self.path.append({
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z,
                'qx': pose.pose.orientation.x,
                'qy': pose.pose.orientation.y,
                'qz': pose.pose.orientation.z,
                'qw': pose.pose.orientation.w
            })

            self.last_pose = pose
            rospy.loginfo("Recorded Pose: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            ))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: {}".format(e))


    def calculate_distance(self, pose1, pose2):
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        return math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    # def save_path_to_file(self, file_path):
    #     with open(file_path, 'w') as f:
    #         json.dump(self.path, f, indent=4)
    #     rospy.loginfo("Path saved to {}".format(file_path))
    #     # rospy.signal_shutdown("Shutting down after saving path")
    #     QApplication.quit()
        
    def save_path_to_file(self, file_path):
        """保存路径信息到文件"""
        with open(file_path, 'w') as f:
            json.dump(self.path, f, indent=4)
        rospy.loginfo("Path saved to {}".format(file_path))
        # Set a flag to indicate saving is complete
        self.is_saving_complete = True
        
    def signal_handler(self, sig, frame):
        rospy.loginfo("Ctrl+C captured, saving the path...")
        self.is_recording = False

    def start_recording(self):
        self.is_recording = True
        self.path = []
        rospy.loginfo("Recording started")

    def stop_recording(self):
        self.is_recording = False
        rospy.loginfo("Recording stopped")
        self.show_save_dialog()

    def show_save_dialog(self):
        app = QApplication([])  
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getSaveFileName(None, "Save Path", "", "JSON Files (*.json)")
        if file_path:
            self.save_path_to_file(file_path)
        app.quit()


class RecorderUI(QMainWindow):
    def __init__(self, recorder):
        super().__init__()
        self.recorder = recorder
        self.initUI()

    def initUI(self):
        self.setWindowTitle('地图采集')
        self.setGeometry(100, 100, 500, 400)

        font = QFont('Arial', 12)

        layout = QVBoxLayout()

        start_button = QPushButton('开始记录')
        start_button.setFont(font)
        start_button.setStyleSheet("background-color: #4CAF50; color: black; padding: 10px;")
        start_button.clicked.connect(self.start_recording)
        layout.addWidget(start_button)

        stop_button = QPushButton('停止记录')
        stop_button.setFont(font)
        stop_button.setStyleSheet("background-color: #f44336; color: black; padding: 10px;")
        stop_button.clicked.connect(self.stop_recording)
        layout.addWidget(stop_button)

        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)

    def start_recording(self):
        self.recorder.start_recording()

    def stop_recording(self):
        self.recorder.stop_recording()


if __name__ == '__main__':
    rospy.init_node('odom_recorder')

    recorder = OdomRecorder()
    recorder.is_saving_complete = False  # Initialize saving complete flag

    ros_thread = threading.Thread(target=rospy.spin)
    ros_thread.start()

    app = QApplication(sys.argv)
    ui = RecorderUI(recorder)
    ui.show()
    # app.exec_()

    sys.exit(app.exec_())
    # rospy.signal_shutdown("Shutting down after saving path")

    # After PyQt application has quit, check if file was saved
    if recorder.is_saving_complete:
        rospy.loginfo("Shutting down ROS after file save.")
        rospy.signal_shutdown("Shutting down after saving file")
        ros_thread.join()  # Ensure the ROS thread is properly shut down