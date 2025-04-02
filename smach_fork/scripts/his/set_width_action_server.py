#!/usr/bin/env python3

'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-09-02 02:31:20
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-09-02 02:59:12
FilePath: /undefined/home/cyun/forklift_sim_ws/src/smach_fork/scripts/set_width_action_server.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import rospy
import actionlib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from smach_fork.msg import SetWidthAction, SetWidthFeedback, SetWidthResult

class SetWidthActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('set_width', SetWidthAction, self.execute, False)
        self.server.start()

        self.pub_left_width = rospy.Publisher('/base_link_to_fork_left_horizontal_controller/command', Float64, queue_size=10)
        self.pub_right_width = rospy.Publisher('/base_link_to_fork_right_horizontal_controller/command', Float64, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.current_width = 0.0  # 初始化当前宽度

    def joint_state_callback(self, msg):
        # 假设宽度与某一特定关节有关
        self.current_width = msg.position[msg.name.index('base_link_to_fork_left_horizontal')]  # 替换为实际关节名称
        # print("Current width:", self.current_width)

    def execute(self, goal):
        feedback = SetWidthFeedback()
        result = SetWidthResult()
        
        rate = rospy.Rate(10)
        success = True

        while not rospy.is_shutdown():
            print("goal.width_goal", goal.width_goal)
            print("error", goal.width_goal - self.current_width)
            print("current_width", self.current_width)
            error = goal.width_goal - self.current_width
            if abs(error) < 0.05:  # 如果接近目标宽度
                result.current_width = self.current_width
                result.success = True
                
                # 发布最终的控制信号并等待1秒
                self.pub_left_width.publish(Float64(goal.width_goal))
                self.pub_right_width.publish(Float64(-goal.width_goal))
                rospy.sleep(1)  # 延迟1秒
                self.server.set_succeeded(result)
                break

            # 计算控制输出（简单比例控制器，P 控制器）
            control_signal = error * 1.0  # 比例系数可以调整

            # 发布控制信号
            self.pub_left_width.publish(Float64(goal.width_goal))
            self.pub_right_width.publish(Float64(-goal.width_goal))


            # 发布反馈
            if goal.width_goal != 0:
                feedback.progress = (self.current_width / goal.width_goal) * 100.0
            else:
                feedback.progress = 100.0  # 如果目标宽度为零，认为任务完成

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('set_width_action_server')
    server = SetWidthActionServer()
    rospy.spin()
