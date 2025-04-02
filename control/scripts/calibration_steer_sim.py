#!/usr/bin/env python
#coding=utf-8
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-20 10:42:06
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-10-20 10:55:21
FilePath: /src/control/scripts/calibration_steer.py
Description: 标定车辆的转向过程

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import rospy
from std_msgs.msg import Float64, UInt8
from nav_msgs.msg import Odometry
import math

class PublisherSubscriber:
    def __init__(self):
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)

        self.steering_msg = Float64(-1.1)
        self.throttle_msg = Float64(0.1)
        self.gear_msg = UInt8(3)
        self.brake_msg = Float64(0.0)

        self.first_odom = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)

    def publish_data(self):
        if self.should_publish_zero():
            self.steering_msg = Float64(0.0)
            self.throttle_msg = Float64(0.0)
            self.gear_msg = UInt8(0)
            self.brake_msg = Float64(0.0)
        self.steering_pub.publish(self.steering_msg)
        self.throttle_pub.publish(self.throttle_msg)
        self.gear_pub.publish(self.gear_msg)
        self.brake_pub.publish(self.brake_msg)

    def odom_callback(self, odom_msg):
        pose = odom_msg.pose.pose
        orientation = pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        print(f"X: {pose.position.x}, Y: {pose.position.y}, Yaw: {yaw}")
        if self.first_odom is None:
            self.first_odom = yaw
        else:
            yaw_error = abs(self.first_odom - yaw)
            if yaw_error >= (90 * math.pi / 180):  # 90 degrees in radians
                self.steering_msg = Float64(0.0)
                self.throttle_msg = Float64(0.0)
                self.gear_msg = UInt8(0)
                self.brake_msg = Float64(0.0)

    def euler_from_quaternion(self, x, y, z, w):
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def should_publish_zero(self):
        return self.first_odom is not None and (abs(self.first_odom - self.euler_from_quaternion(*[0] * 4)[2]) >= (90 * 3.141592653589793 / 180))


if __name__ == '__main__':
    rospy.init_node('my_node')
    ps = PublisherSubscriber()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ps.publish_data()
        rate.sleep()