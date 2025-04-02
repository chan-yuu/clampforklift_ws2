#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import Float64
from smach_fork.msg import SetInitPoseAction, SetInitPoseGoal , SetInitPoseResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class SetInitPoseActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('set_init_pose', SetInitPoseAction, self.execute, False)   
        self.server.start()
        self.pub_init_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    def execute(self, goal):
        
        result = SetInitPoseResult()
        self.pub_init_pose.publish(goal.init_pose)
        time.sleep(3)
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('set_init_pose_action_server')
    server = SetInitPoseActionServer()
    rospy.spin()