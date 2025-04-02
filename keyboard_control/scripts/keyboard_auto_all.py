#!/usr/bin/env python3

import pygame
from pygame.locals import *
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# 初始化pygame
pygame.init()


# 加载图片并获取尺寸
control_image = pygame.image.load("/home/cyun/forklift_sim_ws/src/png/keyboard_control.png")
original_size = control_image.get_size()
image_size = (original_size[0] // 3, original_size[1] // 3)

# 将图片缩小到一半大小
control_image = pygame.transform.scale(control_image, image_size)

# 设置窗口尺寸为缩小后的图片大小
screen = pygame.display.set_mode(image_size, RESIZABLE)
pygame.display.set_caption("Teleop Twist Keyboard")

# 显示提示信息
font = pygame.font.Font(None, 24)
text_color = (255, 255, 255)
instruction_text = "Welcome TO Forklift"

# ROS节点初始化
rospy.init_node('teleop_forklift_keyboard')

# 发布者
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
fork_pub = rospy.Publisher('/fork_controller/command', Float64, queue_size=10)
# joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

# 速度参数
speed = rospy.get_param("~speed", 0.5)
turn = rospy.get_param("~turn", 1.0)

# 初始化变量
x = 0
y = 0
z = 0
th = 0
current_speed = speed
current_turn = turn
fork_position = 0.0
position_step = 0.1  # 每次按键改变的高度
speed_lock = False  # 速度保持状态

key_status = {
    K_w: False,
    K_x: False,
    K_a: False,
    K_d: False,
    K_q: False,
    K_e: False,
    K_z: False,
    K_c: False,
    K_s: False,
    K_UP: False,
    K_DOWN: False
}

def vels(speed, turn):
    return "currently: speed %.2f turning %.2f" % (speed, turn)

def draw_interface():
    # 将背景填充为图片
    screen.blit(control_image, (0, 0))

    # instructions = font.render(instruction_text, True, text_color)
    # instructions_rect = instructions.get_rect(center=(image_size[0] // 2, image_size[1] // 8))
    # screen.blit(instructions, instructions_rect)

    # speed_info = font.render(vels(current_speed, current_turn), True, text_color)
    # speed_info_rect = speed_info.get_rect(center=(image_size[0] // 2, image_size[1] // 4))
    # screen.blit(speed_info, speed_info_rect)

    # 绘制叉头位置条
    max_fork_height = image_size[1] // 2
    fork_height = int(fork_position / 2.0 * max_fork_height)
    if fork_height >= 0:
        pygame.draw.rect(screen, (0, 0, 255), (int(image_size[0] * 0.9), max_fork_height - fork_height, 20, fork_height))
    else:
        pygame.draw.rect(screen, (0, 0, 255), (int(image_size[0] * 0.9), max_fork_height, 20, -fork_height))

    # 显示叉头位置标签
    fork_label_text = "Fork Position"
    fork_label = font.render(fork_label_text, True, (0, 0, 255))
    fork_label_rect = fork_label.get_rect(center=(int(image_size[0] * 0.9) + 10, max_fork_height + 20))
    pygame.draw.rect(screen, (255, 255, 255), fork_label_rect.inflate(10, 10))  # 白底
    pygame.draw.rect(screen, (0, 0, 0), fork_label_rect.inflate(10, 10), 1)  # 天蓝边
    screen.blit(fork_label, fork_label_rect)


    # 绘制速度条
    max_speed_height = image_size[1] // 2
    speed_height = int(x * current_speed / 2.0 * max_speed_height)
    if speed_height >= 0:
        pygame.draw.rect(screen, (0, 0, 255), (int(image_size[0] * 0.1), max_speed_height - speed_height, 20, speed_height))
    else:
        pygame.draw.rect(screen, (0, 0, 255), (int(image_size[0] * 0.1), max_speed_height, 20, -speed_height))

    # 显示速度标签
    speed_label_text = "Speed"
    speed_label = font.render(speed_label_text, True, (0, 0, 255))
    speed_label_rect = speed_label.get_rect(center=(int(image_size[0] * 0.1) + 10, max_speed_height + 20))
    pygame.draw.rect(screen, (255, 255, 255), speed_label_rect.inflate(10, 10))  # 白底
    pygame.draw.rect(screen, (0, 0, 0), speed_label_rect.inflate(10, 10), 1)  # 
    screen.blit(speed_label, speed_label_rect)

    # 绘制转向速度条
    turn_height = int(th * current_turn / 2.0 * max_speed_height)
    if turn_height >= 0:
        pygame.draw.rect(screen, (0, 0, 255), (int(image_size[0] * 0.18), max_speed_height - turn_height, 20, turn_height))
    else:
        pygame.draw.rect(screen, (0, 0, 255), (int(image_size[0] * 0.18), max_speed_height, 20, -turn_height))

    # 显示转向速度标签
    turn_label_text = "Turn"
    turn_label = font.render(turn_label_text, True, (0, 0, 255))
    turn_label_rect = turn_label.get_rect(center=(int(image_size[0] * 0.18) + 10, max_speed_height + 20))
    pygame.draw.rect(screen, (255, 255, 255), turn_label_rect.inflate(10, 10))  # 白底
    pygame.draw.rect(screen, (0, 0, 0), turn_label_rect.inflate(10, 10), 1)  # 
    screen.blit(turn_label, turn_label_rect)


    pygame.display.flip()

# print(vels(current_speed, current_turn))

# 主循环
running = True
fork_updated = False  # 增加一个标志位
while running and not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == VIDEORESIZE:
            image_size = event.size
            screen = pygame.display.set_mode(image_size, RESIZABLE)
        elif event.type == KEYDOWN:
            if event.key in key_status:
                key_status[event.key] = True
                fork_updated = True  # 标记为已更新
                if event.key == K_w:
                    x = 1  # 设置前进速度
                    speed_lock = True  # 启动速度保持
                elif event.key == K_x:
                    x = -1  # 设置后退速度
                    speed_lock = True  # 启动速度保持
                elif event.key == K_s:
                    x = 0  # 清零速度
                    speed_lock = False  # 取消速度保持
            elif event.key == K_ESCAPE:
                running = False
        elif event.type == KEYUP:
            if event.key in key_status:
                key_status[event.key] = False
                fork_updated = True  # 标记为已更新

    # 更新移动方向
    if not speed_lock:
        x = 0

    th = key_status[K_a] - key_status[K_d]
    z = 0

    # 更新速度
    if key_status[K_q]:
        current_speed *= 1.1
        key_status[K_q] = False
    if key_status[K_e]:
        current_speed *= 0.9
        key_status[K_e] = False
    if key_status[K_z]:
        current_turn *= 1.1
        key_status[K_z] = False
    if key_status[K_c]:
        current_turn *= 0.9
        key_status[K_c] = False

    # 创建Twist消息
    twist = Twist()
    twist.linear.x = x * current_speed
    twist.linear.y = 0
    twist.linear.z = z * current_speed
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = th * current_turn

    # 发布cmd_vel消息
    cmd_vel_pub.publish(twist)

    # 更新叉头位置
    if fork_updated:
        if key_status[K_UP] and fork_position < 2.0:
            fork_position += position_step
            fork_position = min(fork_position, 2.0)  # 确保不超过最大值
        elif key_status[K_DOWN] and fork_position > 0.0:
            fork_position -= position_step
            fork_position = max(fork_position, 0.0)  # 确保不低于最小值
        fork_pub.publish(fork_position)
        fork_updated = False  # 重置标志位

    
    # 发布关节状态
    # joint_state = JointState()
    # joint_state.header.stamp = rospy.Time.now()
    # joint_state.name = ['base_to_fork', 'front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
    # joint_state.position = [fork_position, 0.0, 0.0, 0.0, 0.0]
    # joint_pub.publish(joint_state)

    draw_interface()

    # 控制循环频率
    rospy.Rate(50).sleep()

pygame.quit()
