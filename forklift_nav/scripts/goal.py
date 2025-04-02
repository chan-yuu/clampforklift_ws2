#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def publish_goal():
    rospy.init_node('publish_2d_nav_goal')

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # 等待发布器连接
    rospy.sleep(1)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    # 设置位置
    goal.pose.position.x = 9.521446
    goal.pose.position.y = 1.056613
    goal.pose.position.z = 0.059997

    # 将yaw转换为四元数
    quaternion = quaternion_from_euler(0, 0, -0.001078)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]

    # 发布目标
    rospy.loginfo("Publishing goal: %s", goal)
    pub.publish(goal)

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
