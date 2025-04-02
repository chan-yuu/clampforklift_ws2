#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool

# 正常行驶状态机
class NormalDriving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['slow_down', 'avoid_obstacle', 'stop_obstacle', 'normal_driving'])
        self.vehicle_speed = 30.0
        self.distance_to_obstacle = 100.0
        self.obstacle_detected = False

        # 接收的感知信息——车速，障碍物距离，障碍物检测
        rospy.Subscriber('/vehicle_speed', Float32, self.vehicle_speed_callback)
        rospy.Subscriber('/distance_to_obstacle', Float32, self.distance_to_obstacle_callback)
        rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_detected_callback)

    def vehicle_speed_callback(self, msg):
        self.vehicle_speed = msg.data

    def distance_to_obstacle_callback(self, msg):
        self.distance_to_obstacle = msg.data

    def obstacle_detected_callback(self, msg):
        self.obstacle_detected = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: NORMAL DRIVING')
        
        # 状态切换
        if self.obstacle_detected:
            if self.distance_to_obstacle < 5:
                rospy.loginfo('Obstacle too close, switching to STOP OBSTACLE')
                return 'stop_obstacle'
            else:
                rospy.loginfo('Obstacle detected, switching to AVOID OBSTACLE')
                return 'avoid_obstacle'
        elif self.distance_to_obstacle < 10:
            rospy.loginfo('Obstacle nearby, slowing down')
            return 'slow_down'
        else:
            rospy.loginfo('Continuing normal driving')
            return 'normal_driving'

class SlowDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normal_driving'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: SLOW DOWN')
        rospy.sleep(2)
        return 'normal_driving'

class AvoidObstacle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normal_driving'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: AVOID OBSTACLE')
        rospy.sleep(3)  # 模拟避障操作
        return 'normal_driving'

class StopObstacle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normal_driving'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: STOP OBSTACLE')
        rospy.sleep(3)  # 模拟停障操作
        return 'normal_driving'

def main():
    rospy.init_node('vehicle_decision_state_machine')

    sm = smach.StateMachine(outcomes=['outcome4'])

    with sm:
        smach.StateMachine.add('NORMAL_DRIVING', NormalDriving(), 
                               transitions={'slow_down':'SLOW_DOWN', 
                                            'avoid_obstacle':'AVOID_OBSTACLE',
                                            'stop_obstacle':'STOP_OBSTACLE',
                                            'normal_driving':'NORMAL_DRIVING'})
        smach.StateMachine.add('SLOW_DOWN', SlowDown(), 
                               transitions={'normal_driving':'NORMAL_DRIVING'})
        smach.StateMachine.add('AVOID_OBSTACLE', AvoidObstacle(), 
                               transitions={'normal_driving':'NORMAL_DRIVING'})
        smach.StateMachine.add('STOP_OBSTACLE', StopObstacle(), 
                               transitions={'normal_driving':'NORMAL_DRIVING'})

    # 可视化
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()

    # 运行状态机
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()