#!/usr/bin/env python

'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-06 19:35:00
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-10-07 09:56:37
FilePath: /src/clamp_fork/scripts/p3d_to_odom_tf.py
Description: 订阅/odom_p3d的数据（gazebo中直接获取的定位信息），然后将其作为odom和base_footprint的tf发布出去

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''

import rospy
from nav_msgs.msg import Odometry
import tf

def odom_callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_footprint",
                     "odom")
    

if __name__ == '__main__':
    rospy.init_node('odom_tf_broadcaster')
    rospy.Subscriber('/odom_p3d', Odometry, odom_callback)
    rospy.spin()