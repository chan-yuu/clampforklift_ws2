#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import Float64
from smach_fork.msg import SetTarPoseAction, SetTarPoseGoal, SetTarPoseResult, SetTarPoseFeedback
from geometry_msgs.msg import PoseStamped
from car_interfaces.msg import GpsImuInterface
import math

class SetTarPoseActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('set_tar_pose', SetTarPoseAction, self.execute, False)
        self.server.start()

        self.pub_tar_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.sub_map_pose = rospy.Subscriber('map_pose', GpsImuInterface, self.map_pose_callback)
        self.cur_pos_x = 1.0
        self.cur_pos_y = 0.0
        self.tar_x = 0.0
        self.tar_y = 0.0
        self.progress = 0.0

    def map_pose_callback(self, msg):
        self.cur_pos_x = msg.x
        self.cur_pos_y = msg.y

    def execute(self, goal):
        feedback = SetTarPoseFeedback()
        result = SetTarPoseResult()

        # 打印客户端发送的目标姿态信息
        rospy.loginfo("Received target pose:")
        rospy.loginfo("Position: x=%f, y=%f, z=%f", goal.tar_pose.pose.position.x, goal.tar_pose.pose.position.y, goal.tar_pose.pose.position.z)
        rospy.loginfo("Orientation: x=%f, y=%f, z=%f, w=%f", goal.tar_pose.pose.orientation.x, goal.tar_pose.pose.orientation.y, goal.tar_pose.pose.orientation.z, goal.tar_pose.pose.orientation.w)

        rate = rospy.Rate(1)
        self.pub_tar_pose.publish(goal.tar_pose)
        self.tar_x = goal.tar_pose.pose.position.x
        self.tar_y = goal.tar_pose.pose.position.y
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('set_tar_pose_action_server')
    server = SetTarPoseActionServer()
    rospy.spin()