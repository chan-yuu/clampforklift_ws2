#!/usr/bin/env python3
#coding=utf-8

import rospy
from car_interfaces.msg import GpsImuInterface


if __name__ == "__main__":
    rospy.init_node("gps_imu_node")  
    rospy.logwarn("start")

    pub = rospy.Publisher("gps_imu", GpsImuInterface, queue_size = 10) 

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("success")  
        msg = GpsImuInterface()
        msg.PosLon =1
        # msg.PosLan =1
        msg.VelE =2

        # msg.posX =  388652.2938084109
        # msg.posY =  4963429.720356053
        msg.posX =  388624.6230967276
        msg.posY =  4963399.26195412
        msg.Vel = 1
        msg.AngleHeading = 82
        msg.GpsNumSatsUsed = 20

        pub.publish(msg)

        rate.sleep()
