#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import pygame

# 初始化 ROS 节点
rospy.init_node('fork_height_controller', anonymous=True)

# 发布者
pub = rospy.Publisher('/fork_controller/command', Float64, queue_size=10)
joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

# 初始化 pygame
pygame.init()
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption("Fork Height Controller")

# 叉头位置
fork_position = 0.0
position_step = 0.1  # 每次按键改变的高度

# 主循环
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                fork_position += position_step
            elif event.key == pygame.K_DOWN:
                fork_position -= position_step

            # 限制叉头位置范围
            fork_position = max(0.0, min(2.0, fork_position))

            # 发布位置命令
            rospy.loginfo("Setting fork position to: %f", fork_position)
            pub.publish(fork_position)

    # 发布关节状态
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['base_to_fork', 'front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
    joint_state.position = [fork_position, 0.0, 0.0, 0.0, 0.0]
    joint_pub.publish(joint_state)

    # 填充屏幕背景颜色
    screen.fill((255, 255, 255))

    # 绘制叉头位置条
    height = int(fork_position / 2.0 * 480)
    pygame.draw.rect(screen, (0, 0, 255), (300, 480 - height, 40, height))

    # 更新显示
    pygame.display.flip()

    # 控制循环频率
    rospy.Rate(10).sleep()

pygame.quit()
