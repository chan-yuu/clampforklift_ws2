#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-06 22:10:52
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-10-07 09:56:05
FilePath: /src/clamp_fork/scripts/pub_odom.py
Description: 订阅map_to_pose话题的数据，发布odom话题的标准数据

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''


import rospy
import tf
import tf.transformations as tft
import geometry_msgs.msg
from car_interfaces.msg import GpsImuInterface  # 替换为你的ROS包名和消息名
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Quaternion

def callback(msg):
    print("callback map_to_pose")
    # 初始化 Odometry 消息
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "map"

    # 设置 pose
    odom_msg.pose.pose.position = Point(msg.x, msg.y, 0)  # 创建一个 Point 对象
    odom_msg.pose.pose.orientation = Quaternion(0, 0, msg.yaw * math.pi / 180, 1)  # 创建一个 Quaternion 对象

    # 设置 child_frame_id
    odom_msg.child_frame_id = "base_footprint"

    # 发布消息
    odom_pub.publish(odom_msg)
def listener():
    # 初始化节点
    rospy.init_node('odom_publish', anonymous=True)

    global odom_pub
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    # 创建 subscriber，订阅 map_to_pose 话题
    rospy.Subscriber('map_to_base', GpsImuInterface, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()