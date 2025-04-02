#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-06 19:35:00
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-08 17:46:54
FilePath: /scripts/map_to_world.py
Description: 从map、odom和base_footprint的坐标变换中获取最终的定位信息map_to_base话题

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''

import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import numpy as np
import tf


if __name__ == '__main__':
    rospy.init_node('map_to_world_transform')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10)

    # 假设已知的map坐标系下的位置和四元数
    x1, y1, z1 = 1.0, 2.0, 3.0
    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    # 假设已知的world坐标系下的位置和四元数
    x2, y2, z2 = 5.0, 6.0, 7.0
    qx_w, qy_w, qz_w, qw_w = 0.0, 0.0, 0.0, 1.0

    while not rospy.is_shutdown():
        try:
            # 计算平移和旋转
            translation = (x2 - x1, y2 - y1, z2 - z1)
            rotation = tf.transformations.quaternion_multiply((qx_w, qy_w, qz_w, qw_w),
                                                             tf.transformations.quaternion_inverse((qx, qy, qz, qw)))

            point_in_map = geometry_msgs.msg.PointStamped()
            point_in_map.header.frame_id = "map"
            point_in_map.header.stamp = rospy.Time.now()
            point_in_map.point.x = 1.0
            point_in_map.point.y = 2.0
            point_in_map.point.z = 0.0

            transform_stamped = geometry_msgs.msg.TransformStamped()
            transform_stamped.header.frame_id = "world"
            transform_stamped.child_frame_id = "map"
            transform_stamped.transform.translation.x = translation[0]
            transform_stamped.transform.translation.y = translation[1]
            transform_stamped.transform.translation.z = translation[2]
            transform_stamped.transform.rotation.x = rotation[0]
            transform_stamped.transform.rotation.y = rotation[1]
            transform_stamped.transform.rotation.z = rotation[2]
            transform_stamped.transform.rotation.w = rotation[3]
            transform_stamped.header.stamp = rospy.Time.now()

            point_in_world = tf2_geometry_msgs.do_transform_point(point_in_map, transform_stamped)
            print(f"Point in world coordinates: x={point_in_world.point.x}, y={point_in_world.point.y}, z={point_in_world.point.z}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, Exception) as e:
            print(e)
            rate.sleep()
            continue

        rate.sleep()