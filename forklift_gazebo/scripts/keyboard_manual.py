# #!/usr/bin/env python3

# import pygame
# from pygame.locals import *
# import rospy
# from geometry_msgs.msg import Twist

# # 初始化pygame
# pygame.init()

# # 加载图片并获取尺寸
# control_image = pygame.image.load("/home/cyun/robot_ws/files/keyboard_control.png")
# original_size = control_image.get_size()
# image_size = (original_size[0] // 4, original_size[1] // 4)

# # 将图片缩小到一半大小
# control_image = pygame.transform.scale(control_image, image_size)

# # 设置窗口尺寸为缩小后的图片大小
# screen = pygame.display.set_mode(image_size, RESIZABLE)
# pygame.display.set_caption("Teleop Twist Keyboard")


# # 显示提示信息
# font = pygame.font.Font(None, 2)
# text_color = (255, 255, 255)
# # text_color = (0, 0, 0)  # 黑色

# instruction_text = "Use arrow keys or WASD to move & Q/E to increase/decrease speed"

# # ROS节点初始化
# rospy.init_node('teleop_twist_keyboard')
# pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# # 速度参数
# speed = rospy.get_param("~speed", 0.5)
# turn = rospy.get_param("~turn", 1.0)

# # 初始化变量
# x = 0
# y = 0
# z = 0
# th = 0
# current_speed = speed
# current_turn = turn
# speed_lock = False  # 速度保持状态

# key_status = {
#     K_UP: False,
#     K_DOWN: False,
#     K_LEFT: False,
#     K_RIGHT: False,
#     K_w: False,
#     K_s: False,
#     K_a: False,
#     K_d: False,
#     K_q: False,
#     K_e: False
# }

# def vels(speed, turn):
#     return "currently: speed %.2f turning %.2f" % (speed, turn)

# def draw_interface():
#     # 将背景填充为图片
#     screen.blit(control_image, (0, 0))

#     instructions = font.render(instruction_text, True, text_color)
#     instructions_rect = instructions.get_rect(center=(image_size[0] // 2, image_size[1] // 8))
#     screen.blit(instructions, instructions_rect)

#     speed_info = font.render(vels(current_speed, current_turn), True, text_color)
#     speed_info_rect = speed_info.get_rect(center=(image_size[0] // 2, image_size[1] // 4))
#     screen.blit(speed_info, speed_info_rect)

#     pygame.display.flip()

# print(vels(current_speed, current_turn))

# # 主循环
# running = True
# while running and not rospy.is_shutdown():
#     for event in pygame.event.get():
#         if event.type == QUIT:
#             running = False
#         elif event.type == VIDEORESIZE:
#             image_size = event.size
#             screen = pygame.display.set_mode(image_size, RESIZABLE)
#         elif event.type == KEYDOWN:
#             if event.key in key_status:
#                 key_status[event.key] = True
#                 speed_lock = False  # 取消速度保持状态
#             elif event.key == K_ESCAPE:
#                 running = False
#         elif event.type == KEYUP:
#             if event.key in key_status:
#                 key_status[event.key] = False

#     # 更新移动方向
#     x = key_status[K_UP] - key_status[K_DOWN] or key_status[K_w] - key_status[K_s]
#     th = key_status[K_LEFT] - key_status[K_RIGHT] or key_status[K_a] - key_status[K_d]
#     z = 0

#     # 更新速度
#     if key_status[K_q]:
#         current_speed *= 1.1
#         current_turn *= 1.1
#         key_status[K_q] = False
#     if key_status[K_e]:
#         current_speed *= 0.9
#         current_turn *= 0.9
#         key_status[K_e] = False

#     # 如果没有按键按下，保持当前速度
#     if not any(key_status.values()) and not speed_lock:
#         x = 0
#         th = 0

#     # 锁定速度
#     if any(key_status.values()):
#         speed_lock = True

#     # 创建Twist消息
#     twist = Twist()
#     twist.linear.x = x * current_speed
#     twist.linear.y = 0
#     twist.linear.z = z * current_speed
#     twist.angular.x = 0
#     twist.angular.y = 0
#     twist.angular.z = th * current_turn

#     # 发布消息
#     pub.publish(twist)
#     rospy.Rate(50).sleep()
#     rospy.logwarn("Use arrow keys or WASD to move & Q/E to increase/decrease speed")
#     # 绘制界面
#     draw_interface()

# pygame.quit()


import pygame
from pygame.locals import *
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# 初始化pygame
pygame.init()

# 加载图片并获取尺寸
control_image = pygame.image.load("/home/cyun/robot_ws/files/keyboard_control.png")
original_size = control_image.get_size()
image_size = (original_size[0] // 4, original_size[1] // 4)

# 将图片缩小到一半大小
control_image = pygame.transform.scale(control_image, image_size)

# 设置窗口尺寸为缩小后的图片大小
screen = pygame.display.set_mode(image_size, RESIZABLE)
pygame.display.set_caption("Teleop Twist Keyboard")

# 显示提示信息
font = pygame.font.Font(None, 2)
text_color = (255, 255, 255)
# text_color = (0, 0, 0)  # 黑色

instruction_text = "Use arrow keys or WASD to move & Q/E to increase/decrease speed. Use U/J to raise/lower the fork."

# ROS节点初始化
rospy.init_node('teleop_twist_keyboard')
pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pub_fork = rospy.Publisher('/fork_controller/command', Float64, queue_size=1)

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
speed_lock = False  # 速度保持状态
fork_position = Float64()
fork_position.data = 0.0  # 初始位置

key_status = {
    K_UP: False,
    K_DOWN: False,
    K_LEFT: False,
    K_RIGHT: False,
    K_w: False,
    K_s: False,
    K_a: False,
    K_d: False,
    K_q: False,
    K_e: False,
    K_u: False,  # 用于升高叉头
    K_j: False   # 用于降低叉头
}

def vels(speed, turn):
    return "currently: speed %.2f turning %.2f" % (speed, turn)

def draw_interface():
    # 将背景填充为图片
    screen.blit(control_image, (0, 0))

    instructions = font.render(instruction_text, True, text_color)
    instructions_rect = instructions.get_rect(center=(image_size[0] // 2, image_size[1] // 8))
    screen.blit(instructions, instructions_rect)

    speed_info = font.render(vels(current_speed, current_turn), True, text_color)
    speed_info_rect = speed_info.get_rect(center=(image_size[0] // 2, image_size[1] // 4))
    screen.blit(speed_info, speed_info_rect)

    pygame.display.flip()

print(vels(current_speed, current_turn))

# 主循环
running = True
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
                speed_lock = False  # 取消速度保持状态
            elif event.key == K_ESCAPE:
                running = False
        elif event.type == KEYUP:
            if event.key in key_status:
                key_status[event.key] = False

    # 更新移动方向
    x = key_status[K_UP] - key_status[K_DOWN] or key_status[K_w] - key_status[K_s]
    th = key_status[K_LEFT] - key_status[K_RIGHT] or key_status[K_a] - key_status[K_d]
    z = 0

    # 更新速度
    if key_status[K_q]:
        current_speed *= 1.1
        current_turn *= 1.1
        key_status[K_q] = False
    if key_status[K_e]:
        current_speed *= 0.9
        current_turn *= 0.9
        key_status[K_e] = False

    # 更新叉头位置
    if key_status[K_u]:  # 升高叉头
        fork_position.data += 0.01
        pub_fork.publish(fork_position)
    if key_status[K_j]:  # 降低叉头
        fork_position.data -= 0.01
        pub_fork.publish(fork_position)

    # 如果没有按键按下，保持当前速度
    if not any(key_status.values()) and not speed_lock:
        x = 0
        th = 0

    # 锁定速度
    if any(key_status.values()):
        speed_lock = True

    # 创建Twist消息
    twist = Twist()
    twist.linear.x = x * current_speed
    twist.linear.y = 0
    twist.linear.z = z * current_speed
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = th * current_turn

    # 发布消息
    pub_twist.publish(twist)
    rospy.Rate(50).sleep()
    rospy.logwarn("Use arrow keys or WASD to move & Q/E to increase/decrease speed. Use U/J to raise/lower the fork.")
    # 绘制界面
    draw_interface()

pygame.quit()