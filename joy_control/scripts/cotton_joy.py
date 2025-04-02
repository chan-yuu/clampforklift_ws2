#!/usr/bin/env python3
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-08-02 23:36:58
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-07 11:04:58
FilePath: /forklift_sim_ws3/src/joy_control/scripts/cotton_joy.py
Description: 夹抱车的joy控制

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# import pygame
# from pygame.locals import *
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

twist = Twist()
gain = 0.5
fork_position = -0.3

def callback(data):
    global twist
    global fork_position
    global left_horizontal_target
    global left_vertical_target
    global right_horizontal_target
    global right_vertical_target
    
    gain_danamic_linear = gain #+ abs(data.axes[1])
    gain_danamic_angular = 1 # gain + abs(data.axes[3])

    twist.linear.x = data.axes[1] * gain_danamic_linear # 最大速度2m/s
    twist.angular.z = data.axes[3] * gain_danamic_angular # 最大转向1rad/s
    # 后轮转向反向发送（主要是方便习惯左右，和遥控器相同）
    # twist.angular.z = -data.axes[3] * gain_danamic_angular # 最大转向1rad/s

    if data.buttons[3] == 1:
        fork_position += 0.05
        left_vertical_target += 0.05
        right_vertical_target += 0.05
        # rospy.loginfo("UP FORK!!")
    elif data.buttons[0] == 1:
        fork_position -= 0.05
        left_vertical_target -= 0.05
        right_vertical_target -= 0.05
        # rospy.loginfo("DOWN FORK!!")
    elif data.buttons[1] == 1: #B键
        fork_position -= 0.05
        left_horizontal_target += 0.05
        right_horizontal_target -= 0.05
        # rospy.loginfo("DOWN FORK!!")
    elif data.buttons[2] == 1:
        fork_position -= 0.05
        left_horizontal_target -= 0.05
        right_horizontal_target += 0.05
        # rospy.loginfo("DOWN FORK!!")

    # fork_position = min(fork_position, 3.0)  # 确保不超过最大值
    # fork_position = max(fork_position, -0.4)  # 确保不低于最小值
    right_vertical_target = min(max(right_vertical_target, -1.0), 3.5)
    left_vertical_target = min(max(left_vertical_target, -1.0), 3.5)
    right_horizontal_target = min(max(right_horizontal_target, -0.6), 0.1)
    left_horizontal_target = min(max(left_horizontal_target, -0.1), 0.6)

    # rospy.loginfo('speed_not: %.2f, turn_not: %.2f', data.axes[1], data.axes[3])
    rospy.loginfo('speed: %.2f, turn: %.2f', twist.linear.x, twist.angular.z)
    rospy.loginfo('position: %.2f', fork_position)

    # if data.buttons[2] == 1:
    #     twist.linear.x = 0
    #     twist.angular.z = 0
    #     rospy.loginfo("STOP!!")

# 定义目标位置
left_horizontal_target = 0.0
left_vertical_target = 0.0
right_horizontal_target = -0.0
right_vertical_target = 0.0
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
    left_fork_pub = rospy.Publisher('/base_link_to_fork_left_controller/command', Float64, queue_size=10)
    right_fork_pub = rospy.Publisher('/base_link_to_fork_right_controller/command', Float64, queue_size=10)
    pub_left_horizontal = rospy.Publisher('/base_link_to_fork_left_horizontal_controller/command', Float64, queue_size=10)
    pub_left_vertical = rospy.Publisher('/fork_left_horizontal_to_fork_left_vertical_controller/command', Float64, queue_size=10)
    pub_right_horizontal = rospy.Publisher('/base_link_to_fork_right_horizontal_controller/command', Float64, queue_size=10)
    pub_right_vertical = rospy.Publisher('/fork_right_horizontal_to_fork_right_vertical_controller/command', Float64, queue_size=10)


    rospy.Subscriber("joy", Joy, callback)
    # rospy.spin() 会阻塞当前线程，直到节点退出！！

    while not rospy.is_shutdown():
        for pub in pub_list:
            pub.publish(twist)
        # left_fork_pub.publish(fork_position)
        # right_fork_pub.publish(fork_position)
        pub_left_horizontal.publish(left_horizontal_target)
        pub_left_vertical.publish(left_vertical_target)
        pub_right_horizontal.publish(right_horizontal_target)
        pub_right_vertical.publish(right_vertical_target)
        print("Fork Position:", fork_position)
        print("Horizontal Position:", left_horizontal_target)
        # print("linear speed:", twist.linear.x)
        # print("angular speed:", twist.angular.z)
        rospy.Rate(10).sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass