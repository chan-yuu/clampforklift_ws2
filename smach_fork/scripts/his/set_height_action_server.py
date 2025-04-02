#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from smach_fork.msg import SetHeightAction, SetHeightFeedback, SetHeightResult

class SetHeightActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('set_height', SetHeightAction, self.execute, False)
        self.server.start()

        self.pub_left_vertical = rospy.Publisher('/fork_left_horizontal_to_fork_left_vertical_controller/command', Float64, queue_size=10)
        self.pub_right_vertical = rospy.Publisher('/fork_right_horizontal_to_fork_right_vertical_controller/command', Float64, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.current_height = 0.0  # 初始化当前高度

    def joint_state_callback(self, msg):
        # 假设垂直高度与某一特定关节有关
        self.current_height = msg.position[msg.name.index('fork_left_horizontal_to_fork_left_vertical')]  # 替换为实际关节名称

    def execute(self, goal):
        feedback = SetHeightFeedback()
        result = SetHeightResult()
        
        rate = rospy.Rate(10)
        success = True

        while not rospy.is_shutdown():
            print("goal.height_goal", goal.height_goal)
            print("error", goal.height_goal - self.current_height)
            print("Current height:", self.current_height)

            error = goal.height_goal - self.current_height
            if abs(error) < 0.01:  # 如果接近目标高度
                result.current_height = self.current_height
                result.success = True

                # 发布最终的控制信号并等待1秒
                self.pub_left_vertical.publish(Float64(goal.height_goal))
                self.pub_right_vertical.publish(Float64(goal.height_goal))
                rospy.sleep(1)  # 延迟1秒

                self.server.set_succeeded(result)
                break

            # 计算控制输出（简单比例控制器，P 控制器）
            control_signal = error * 1.0  # 比例系数可以调整    

            # 发布控制信号
            # self.pub_left_vertical.publish(Float64(control_signal))
            # self.pub_right_vertical.publish(Float64(control_signal))
            self.pub_left_vertical.publish(Float64(goal.height_goal))
            self.pub_right_vertical.publish(Float64(goal.height_goal))

            # 发布反馈
            feedback.progress = (self.current_height / goal.height_goal) * 100.0
            self.server.publish_feedback(feedback)
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('set_height_action_server')
    server = SetHeightActionServer()
    rospy.spin()
