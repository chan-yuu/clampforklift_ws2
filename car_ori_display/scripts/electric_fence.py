#!/usr/bin/env python

'''
Author: CYUN && cyun@tju.enu.cn
Date: 2025-01-11 15:09:08
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-15 02:34:59
FilePath: /undefined/home/cyun/forklift_sim_ws3/src/car_ori_display/scripts/electric_fence.py
Description: 电子围栏

Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
'''
#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math


class ElectricFenceMonitor:
    def __init__(self):
        rospy.init_node('electric_fence_publisher', anonymous=True)
        self.pub = rospy.Publisher('electric_fence', MarkerArray, queue_size=10)
        self.brake_pub = rospy.Publisher('brake_cmd', Float64, queue_size=10)
        self.throttle_pub = rospy.Publisher('throttle_cmd', Float64, queue_size=10)
        self.callback_count = 0  # 用于计数回调次数

        # 加载并解析来自参数服务器的电子围栏区域
        fence_params = rospy.get_param('fence_regions')
        self.fence_regions = [[Point(x=p[0], y=p[1], z=p[2]) for p in region] for region in fence_params]


        rospy.Subscriber('dis_odom', Odometry, self.callback)
        self.rate = rospy.Rate(1)

    def is_point_inside_polygon(self, point, polygon):
        x, y = point.x, point.y
        inside = False
        for i in range(len(polygon)):
            j = (i + 1) % len(polygon)
            xi, yi = polygon[i].x, polygon[i].y
            xj, yj = polygon[j].x, polygon[j].y
            intersect = ((yi > y)!= (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            inside = inside!= intersect
        return inside

    def publish_fence(self):
        while not rospy.is_shutdown():
            marker_array = MarkerArray()

            for idx, points in enumerate(self.fence_regions):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "fence_markers"
                marker.id = idx
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD

                # 黄色
                marker.color.a = 1.0  # 不透明
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker.scale.x = 0.3  # 线宽

                marker.points = points

                marker_array.markers.append(marker)

            self.pub.publish(marker_array)
            self.rate.sleep()

    def callback(self, odom_msg):
        current_point = odom_msg.pose.pose.position
        in_fence = False
        for region in self.fence_regions:
            if self.is_point_inside_polygon(current_point, region):
                in_fence = True
                break
        self.check_fence_status(in_fence)
        self.callback_count += 1  # 每次回调后计数器加 1
        if self.callback_count % 100 == 0:  # 每隔 100
            self.callback_count = 0
            self.print_fence_status(in_fence)

    def check_fence_status(self, in_fence):
        if in_fence:
            # self.throttle_pub(0.5) # 测试是否刹车了
            self.brake_pub.publish(6.0)  # 发布刹车指令

    def print_fence_status(self, in_fence):
        if in_fence:
            rospy.loginfo("\033[31m进入电子围栏\033[0m")
        else:
            rospy.loginfo("\033[32m没有进入电子围栏\033[0m")


if __name__ == '__main__':
    try:
        monitor = ElectricFenceMonitor()
        monitor.publish_fence()
    except rospy.ROSInterruptException:
        pass