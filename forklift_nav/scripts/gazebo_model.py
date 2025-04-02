'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-08-02 14:07:25
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-08-02 14:07:52
FilePath: /CM13_Forklift2024/home/cyun/forklift_sim_ws/src/forklift_nav/scripts/gazebo_model.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates

def model_states_callback(data):
    for i in range(len(data.name)):
        model_name = data.name[i]
        position = data.pose[i].position
        orientation = data.pose[i].orientation
        rospy.loginfo("Model: %s", model_name)
        rospy.loginfo("  Position -> x: %.2f, y: %.2f, z: %.2f", position.x, position.y, position.z)
        rospy.loginfo("  Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f", orientation.x, orientation.y, orientation.z, orientation.w)

def listener():
    rospy.init_node('gazebo_model_listener', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
