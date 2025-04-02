'''
Author: CYUN && cyun@tju.enu.cn
Date: 2025-01-10 16:51:53
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-10 16:52:13
FilePath: /undefined/home/cyun/forklift_sim_ws3/src/smach_fork/scripts/test_smach.py
Description: 

Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
'''
import rospy
import smach
import smach_ros


class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Starting the robot...')
        # 这里可以添加实际启动机器人的代码，比如发送启动指令给硬件驱动
        return'success'


class MoveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Moving the robot...')
        # 假设这里是移动逻辑，例如发布速度控制指令
        return'success'


class StopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Stopping the robot...')
        # 放置停止机器人的代码，如发送停止指令
        return'success'


def main():
    rospy.init_node('robot_state_machine')

    # 创建状态机
    sm = smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('START', StartState(), transitions={'success': 'MOVE'})
        smach.StateMachine.add('MOVE', MoveState(), transitions={'success': 'STOP'})
        smach.StateMachine.add('STOP', StopState(), transitions={'success': 'finished'})

    # 创建并启动状态机的内省服务器
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # 执行状态机
    outcome = sm.execute()

    # 等待内省服务器结束
    sis.stop()

    rospy.loginfo('State machine outcome: %s', outcome)


if __name__ == '__main__':
    main()