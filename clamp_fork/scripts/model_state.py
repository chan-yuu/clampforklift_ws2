#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion


def set_model_state():
    rospy.init_node('set_model_state_publisher')

    set_state = rospy.Publisher(
        "gazebo/set_model_state", ModelState, queue_size=10
    )
    

    rate = rospy.Rate(1)  # 设置发布频率，每秒发布一次

    model_state = ModelState()
    model_state.model_name = "smart"

    new_pose = Pose()
    new_pose.position = Point(x=2.0, y=0.0, z=0.0)
    new_pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
    model_state.pose = new_pose
    

    while not rospy.is_shutdown():
        print(1111)
        set_state.publish(model_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        set_model_state()
    except rospy.ROSInterruptException:
        pass