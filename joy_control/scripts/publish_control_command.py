#!/usr/bin/env python
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-08-03 01:12:13
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-08-03 18:50:21
FilePath: /src/joy_control/scripts/publish_control_command.py
Description: 控制夹抱车的夹抱机器

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import pygame

# 定义目标位置
left_horizontal_target = 0.15
left_vertical_target = 1.0
right_horizontal_target = -0.15
right_vertical_target = 1.0

# 定义容器存储关节位置
current_positions = {
    'base_link_to_fork_left_horizontal': 0.0,
    'fork_left_horizontal_to_fork_left_vertical': 0.0,
    'base_link_to_fork_right_horizontal': 0.0,
    'fork_right_horizontal_to_fork_right_vertical': 0.0
}

# 回调函数，用于处理从 /joint_states 获取的数据
def joint_states_callback(data):
    global current_positions
    for i, name in enumerate(data.name):
        if name in current_positions:
            current_positions[name] = data.position[i]

def publish_control_commands():
    global left_horizontal_target, left_vertical_target, right_horizontal_target, right_vertical_target
    
    rospy.init_node('control_command_publisher', anonymous=True)
    
    # 创建发布器
    pub_left_horizontal = rospy.Publisher('/base_link_to_fork_left_horizontal_controller/command', Float64, queue_size=10)
    pub_left_vertical = rospy.Publisher('/fork_left_horizontal_to_fork_left_vertical_controller/command', Float64, queue_size=10)
    pub_right_horizontal = rospy.Publisher('/base_link_to_fork_right_horizontal_controller/command', Float64, queue_size=10)
    pub_right_vertical = rospy.Publisher('/fork_right_horizontal_to_fork_right_vertical_controller/command', Float64, queue_size=10)
    
    # 订阅 /joint_states 话题
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    
    rate = rospy.Rate(10)  # 10 Hz

    # 初始化 pygame
    pygame.init()
    screen = pygame.display.set_mode((100, 100))

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    left_vertical_target += 0.05
                    right_vertical_target += 0.05
                elif event.key == pygame.K_DOWN:
                    left_vertical_target -= 0.05
                    right_vertical_target -= 0.05
                elif event.key == pygame.K_RIGHT:
                    left_horizontal_target += 0.05
                    right_horizontal_target -= 0.05
                elif event.key == pygame.K_LEFT:
                    left_horizontal_target -= 0.05
                    right_horizontal_target += 0.05
            elif event.type == pygame.QUIT:
                pygame.quit()
                return

        # 发布控制命令
        pub_left_horizontal.publish(left_horizontal_target)
        pub_left_vertical.publish(left_vertical_target)
        pub_right_horizontal.publish(right_horizontal_target)
        pub_right_vertical.publish(right_vertical_target)
        print("height: ", left_vertical_target)
        print("width: ", abs(right_horizontal_target))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_control_commands()
    except rospy.ROSInterruptException:
        pass
