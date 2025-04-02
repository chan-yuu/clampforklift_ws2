#!/usr/bin/env python
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-07-30 16:41:51
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-08-02 01:08:28
FilePath: /CM13_Forklift2024/home/cyun/forklift_sim_ws/src/joy_control/scripts/pub_joy.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import pygame
from pygame.locals import *
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

twist = Twist()

gain = 1
fork_position = 0.0
def callback(data):
    global twist
    global fork_position
    
    gain_danamic_linear = gain + abs(data.axes[1])
    gain_danamic_angular = gain + abs(data.axes[3])

    twist.linear.x = data.axes[1] * gain_danamic_linear
    twist.angular.z = data.axes[3] * gain_danamic_angular

    if data.buttons[3] == 1:
        fork_position += 0.05
        rospy.loginfo("UP FORK!!")
    elif data.buttons[0] == 1:
        fork_position -= 0.05
        rospy.loginfo("DOWN FORK!!")

    fork_position = min(fork_position, 2.0)  # 确保不超过最大值
    fork_position = max(fork_position, 0.0)  # 确保不低于最小值

    # rospy.loginfo('speed_not: %.2f, turn_not: %.2f', data.axes[1], data.axes[3])
    rospy.loginfo('speed: %.2f, turn: %.2f', twist.linear.x, twist.angular.z)
    rospy.loginfo('position: %.2f', fork_position)

    if data.buttons[2] == 1:
        twist.linear.x = 0
        twist.angular.z = 0
        rospy.loginfo("STOP!!")

fork_position = 0
def main():
    global pub
    global pub_turtle
    global pub_list
    global fork_position
    
    rospy.init_node('joy_control', anonymous=True)

    pub_list = [
        rospy.Publisher('/cmd_vel', Twist, queue_size=10),
        rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10),
        # rospy.Publisher('cmd_vel', Twist, queue_size=1),
    ]
    fork_pub = rospy.Publisher('/fork_controller/command', Float64, queue_size=10)
    left_fork_pub = rospy.Publisher('/base_link_to_fork_left_controller/command', Float64, queue_size=10)
    right_fork_pub = rospy.Publisher('/base_link_to_fork_right_controller/command', Float64, queue_size=10)

    
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # pub_turtle = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)
    # rospy.spin() 会阻塞当前线程，直到节点退出！！

    while not rospy.is_shutdown():
        # pub.publish(twist)
        # pub_turtle.publish(twist)
        for pub in pub_list:
            pub.publish(twist)
        fork_pub.publish(fork_position)
        left_fork_pub.publish(fork_position)
        right_fork_pub.publish(fork_position)
        rospy.Rate(10).sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass