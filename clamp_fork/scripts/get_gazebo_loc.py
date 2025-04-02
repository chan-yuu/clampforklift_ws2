#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-09-04 00:04:26
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-08 17:35:23
FilePath: /undefined/home/cyun/forklift_sim_ws3/src/clamp_fork/scripts/get_gazebo_loc.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''


import rospy
from gazebo_msgs.msg import ModelStates

def callback(data):
    # 获取模型的名称和位置
    try:
        model_index = data.name.index('your_model_name')
        position = data.pose[model_index].position
        rospy.loginfo(f"Model Position: x={position.x}, y={position.y}, z={position.z}")
    except ValueError:
        rospy.logwarn("Model not found!")

def listener():
    rospy.init_node('gazebo_model_listener', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()