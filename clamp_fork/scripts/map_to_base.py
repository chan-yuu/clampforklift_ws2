#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-06 19:35:00
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-08 17:48:00
FilePath: /scripts/map_to_base.py
Description: 从map、odom和base_footprint的坐标变换中获取最终的定位信息map_to_base话题

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''

import rospy
import tf
import geometry_msgs.msg
from car_interfaces.msg import GpsImuInterface  # 替换为你定义的消息类型
import math

class MapToBasePublisher:
    def __init__(self):
        rospy.init_node('map_to_base_publisher')

        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)  # 等待 TF 缓冲区准备好

        self.map_to_base_pub = rospy.Publisher('/map_to_base', GpsImuInterface, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz 的频率发布

    def publish_transform(self):
        while not rospy.is_shutdown():
            try:
                # print("pub ok")
                # NOTO 这里不能直接获取map和base_footprint的变换吗？？
                # 获取 map 到 odom 的变换
                (trans_map_to_odom, rot_map_to_odom) = self.tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
                # 获取 odom 到 base_footprint 的变换
                (trans_odom_to_base, rot_odom_to_base) = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
                
                # 将平移和旋转分别转换为矩阵形式
                mat_map_to_odom = tf.transformations.concatenate_matrices(
                    tf.transformations.translation_matrix(trans_map_to_odom),
                    tf.transformations.quaternion_matrix(rot_map_to_odom)
                )
                
                mat_odom_to_base = tf.transformations.concatenate_matrices(
                    tf.transformations.translation_matrix(trans_odom_to_base),
                    tf.transformations.quaternion_matrix(rot_odom_to_base)
                )

                # 组合两个矩阵以获得最终的变换矩阵
                mat_final = tf.transformations.concatenate_matrices(mat_map_to_odom, mat_odom_to_base)

                # 从最终的变换矩阵提取平移和旋转
                trans_final = tf.transformations.translation_from_matrix(mat_final)
                rot_final = tf.transformations.quaternion_from_matrix(mat_final)
                
                # 将四元数转换为欧拉角
                euler = tf.transformations.euler_from_quaternion(rot_final)
                
                # 创建并填充自定义消息
                msg = GpsImuInterface()
                msg.x = trans_final[0]
                msg.y = trans_final[1]
                msg.z = trans_final[2]
                msg.roll = euler[0]
                msg.pitch = euler[1]
                msg.yaw = euler[2] * 180 / math.pi

                # 发布消息
                self.map_to_base_pub.publish(msg)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("TF lookup failed: {}".format(e))
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        map_to_base_publisher = MapToBasePublisher()
        map_to_base_publisher.publish_transform()
    except rospy.ROSInterruptException:
        pass
