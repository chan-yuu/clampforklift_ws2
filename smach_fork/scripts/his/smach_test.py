#!/usr/bin/env python
# import rospy
# import actionlib
# from smach_fork.msg import SetInitPoseAction
# from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
# from smach_fork.msg import SetInitPoseAction, SetInitPoseGoal , SetInitPoseResult
# # 也就是服务端和客户端才是这种单向的，只有客户端请求了服务服务端才会响应，至于这个响应的情况如何也是客户端来判断的
# def feedback_callback(feedback):
#     rospy.loginfo("Feedback: %s" % feedback)

# def main():
#     # 初始化ROS节点
#     rospy.init_node('set_init_pose_client')

#     # 创建动作客户端
#     client = actionlib.SimpleActionClient('set_init_pose', SetInitPoseAction)

#     # 等待动作服务器启动
#     client.wait_for_server()

#     # 创建并发送目标
#     goal = SetInitPoseGoal()
#     goal.init_pose.header.stamp = rospy.Time.now()
#     goal.init_pose.header.frame_id = "map"
#     goal.init_pose.pose.pose = Pose(Point(13.3081, 3.0509, 0.0), Quaternion(0.0, 0.0, 1.0000, 0.0063))
#     client.send_goal(goal)

#     # 等待动作结果
#     client.wait_for_result()

#     if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
#         rospy.loginfo("Action completed!")
#     else:
#         rospy.loginfo("Action failed!")

# if __name__ == '__main__':
#     main()


# 全局规划任务应该重新用服务来接受每个规划需要的点（但是也保留通过topic直接获取点位的方式）

# #!/usr/bin/env python3
# import rospy
# import actionlib
# from std_msgs.msg import Float64
# from smach_fork.msg import SetInitPoseAction, SetInitPoseGoal , SetInitPoseResult
# from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import PoseWithCovarianceStamped
# import time

# class SetInitPoseActionServer:
#     def __init__(self):
#         self.server = actionlib.SimpleActionServer('set_init_pose', SetInitPoseAction, self.execute, False)   
#         self.server.start()
#         self.pub_init_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

#     def execute(self, goal):
        
#         result = SetInitPoseResult()
#         self.pub_init_pose.publish(goal.init_pose)
#         time.sleep(3)
#         result.success = True
#         self.server.set_succeeded(result)

# if __name__ == '__main__':
#     rospy.init_node('set_init_pose_action_server')
#     server = SetInitPoseActionServer()
#     rospy.spin()


#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from smach_fork.msg import SetTarPoseAction, SetTarPoseGoal

def main():
    # 初始化ROS节点
    rospy.init_node('set_tar_pose_action_client')

    # 创建动作客户端
    client = actionlib.SimpleActionClient('set_tar_pose', SetTarPoseAction)
    client.wait_for_server()

    # 构建目标姿态
    target_pose = PoseStamped()
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = "map"
    target_pose.pose.position.x = 10.0  # 示例目标位置
    target_pose.pose.position.y = 5.0
    target_pose.pose.position.z = 0.0
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0

    # 创建并发送目标
    goal = SetTarPoseGoal()
    goal.tar_pose = target_pose
    client.send_goal(goal)

    # 等待结果
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Target pose action completed!")
    else:
        rospy.loginfo("Target pose action failed!")
        
        
    # 创建动作客户端
    client = actionlib.SimpleActionClient('set_tar_pose', SetTarPoseAction)
    client.wait_for_server()

    # 构建目标姿态
    target_pose = PoseStamped()
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = "map"
    target_pose.pose.position.x = 1.0  # 示例目标位置
    target_pose.pose.position.y = 1.0
    target_pose.pose.position.z = 1.0
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0

    # 创建并发送目标
    goal = SetTarPoseGoal()
    goal.tar_pose = target_pose
    client.send_goal(goal)

    # 等待结果
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Target pose action completed!")
    else:
        rospy.loginfo("Target pose action failed!")

if __name__ == '__main__':
    main()