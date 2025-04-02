#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import tf.transformations as tft
import geometry_msgs.msg
from car_interfaces.msg import GpsImuInterface  # 替换为你的ROS包名和消息名
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

def callback(msg):
    # 初始化 TwistStamped 消息
    twist_msg = TwistStamped()
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "base_footprint"

    # 设置 TwistStamped 的 twist.linear.x
    twist_msg.twist.linear.x = msg.twist.twist.linear.x

    # 发布消息
    twist_pub.publish(twist_msg)
    rospy.loginfo("Published to /twist: linear.x: %f", msg.twist.twist.linear.x)

def listener():
    # 初始化节点
    rospy.init_node('twist_publisher', anonymous=True)

    # 创建 publisher，发布到 /twist 话题
    global twist_pub
    twist_pub = rospy.Publisher('twist', TwistStamped, queue_size=10)

    # 创建 subscriber，订阅 /odom_p3d 话题
    rospy.Subscriber('odom_p3d', Odometry, callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    listener()