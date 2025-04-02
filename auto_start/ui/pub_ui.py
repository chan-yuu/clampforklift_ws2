#!/usr/bin/env python
'''
Author: cyun
Date: 2024-09-06 22:32:39
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-09-12 15:55:23
FilePath: /undefined/home/kemove/forklift_sim_ws2/src/forklift_nav/scripts/pub_ui.py
Description: 

Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
'''
import rospy
import json
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QApplication, QFileDialog


class PathPublisher:
    def __init__(self):
        rospy.init_node('path_publisher')
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.path_msg = Path()

    def load_path_from_file(self, file_path):
        """从文件中加载路径"""
        try:
            with open(file_path, 'r') as f:
                path_data = json.load(f)
            rospy.loginfo("Loaded path from file: {}".format(file_path))

            self.path_msg.header.frame_id = "map"
            for pose_data in path_data:
                pose = PoseStamped()
                pose.pose.position.x = pose_data['x']
                pose.pose.position.y = pose_data['y']
                pose.pose.position.z = pose_data['z']
                pose.pose.orientation.x = pose_data['qx']
                pose.pose.orientation.y = pose_data['qy']
                pose.pose.orientation.z = pose_data['qz']
                pose.pose.orientation.w = pose_data['qw']
                self.path_msg.poses.append(pose)

        except Exception as e:
            rospy.logerr("Failed to load path from file: {}".format(e))

    def publish_path(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.path_msg.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.path_msg)
            rate.sleep()

    def show_load_dialog(self):
        """弹出选择路径文件的对话框"""
        app = QApplication([])  # 需要创建 QApplication 实例
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getOpenFileName(None, "Load Path", "", "JSON Files (*.json)")
        if file_path:
            self.load_path_from_file(file_path)
            rospy.loginfo("Path loaded and ready to publish")
        app.quit()


if __name__ == '__main__':
    try:
        publisher = PathPublisher()

        # 打开文件选择对话框并加载路径
        publisher.show_load_dialog()

        # 开始发布路径
        publisher.publish_path()

    except rospy.ROSInterruptException:
        pass
