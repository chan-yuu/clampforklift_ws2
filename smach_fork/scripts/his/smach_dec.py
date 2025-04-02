#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float32

# 空闲状态
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['approaching'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: IDLE')
        rospy.sleep(1)  # 模拟等待指令
        return 'approaching'

# 接近状态
class Approaching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasping', 'emergency'])
        self.position_adjusted = False
        self.emergency_detected = False

        # 接收的感知信息——位置调整完成，紧急情况检测
        rospy.Subscriber('/position_adjusted', Bool, self.position_adjusted_callback)
        rospy.Subscriber('/emergency_detected', Bool, self.emergency_detected_callback)

    def position_adjusted_callback(self, msg):
        self.position_adjusted = msg.data

    def emergency_detected_callback(self, msg):
        self.emergency_detected = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: APPROACHING')
        
        # 状态切换
        if self.emergency_detected:
            rospy.loginfo('Emergency detected, switching to EMERGENCY')
            return 'emergency'
        elif self.position_adjusted:
            rospy.loginfo('Position adjusted, switching to GRASPING')
            return 'grasping'
        else:
            rospy.loginfo('Continuing approaching')
            rospy.sleep(1)  # 模拟接近过程
            return 'grasping'  # 继续接近直到调整完成

# 夹取状态
class Grasping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lifting', 'emergency','grasping'])
        self.grasped = True
        self.emergency_detected = False

        # 接收的感知信息——夹取成功，紧急情况检测
        rospy.Subscriber('/grasped', Bool, self.grasped_callback)
        rospy.Subscriber('/emergency_detected', Bool, self.emergency_detected_callback)

    def grasped_callback(self, msg):
        self.grasped = msg.data

    def emergency_detected_callback(self, msg):
        self.emergency_detected = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: GRASPING')
        
        # 状态切换
        if self.emergency_detected:
            rospy.loginfo('Emergency detected, switching to EMERGENCY')
            return 'emergency'
        elif self.grasped:
            rospy.loginfo('Grasped successfully, switching to LIFTING')
            return 'lifting'
        else:
            rospy.loginfo('Continuing grasping')
            rospy.sleep(1)  # 模拟夹取过程
            return 'grasping'

# 提升状态
class Lifting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['transporting', 'emergency','lifting'])
        self.lifted = True
        self.emergency_detected = False

        # 接收的感知信息——提升完成，紧急情况检测
        rospy.Subscriber('/lifted', Bool, self.lifted_callback)
        rospy.Subscriber('/emergency_detected', Bool, self.emergency_detected_callback)

    def lifted_callback(self, msg):
        self.lifted = msg.data

    def emergency_detected_callback(self, msg):
        self.emergency_detected = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: LIFTING')
        
        # 状态切换
        if self.emergency_detected:
            rospy.loginfo('Emergency detected, switching to EMERGENCY')
            return 'emergency'
        elif self.lifted:
            rospy.loginfo('Lifted successfully, switching to TRANSPORTING')
            return 'transporting'
        else:
            rospy.loginfo('Continuing lifting')
            rospy.sleep(1)  # 模拟提升过程
            return 'lifting'

# 运输状态
class Transporting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['depositing', 'emergency','transporting'])
        self.arrived = True
        self.emergency_detected = False

        # 接收的感知信息——到达目标位置，紧急情况检测
        rospy.Subscriber('/arrived', Bool, self.arrived_callback)
        rospy.Subscriber('/emergency_detected', Bool, self.emergency_detected_callback)

    def arrived_callback(self, msg):
        self.arrived = msg.data

    def emergency_detected_callback(self, msg):
        self.emergency_detected = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: TRANSPORTING')
        
        # 状态切换
        if self.emergency_detected:
            rospy.loginfo('Emergency detected, switching to EMERGENCY')
            return 'emergency'
        elif self.arrived:
            rospy.loginfo('Arrived at destination, switching to DEPOSITING')
            return 'depositing'
        else:
            rospy.loginfo('Continuing transporting')
            rospy.sleep(1)  # 模拟运输过程
            return 'transporting'

# 放置状态
class Depositing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['returning', 'emergency','depositing'])
        self.deposited = True
        self.emergency_detected = False

        # 接收的感知信息——放置完成，紧急情况检测
        rospy.Subscriber('/deposited', Bool, self.deposited_callback)
        rospy.Subscriber('/emergency_detected', Bool, self.emergency_detected_callback)

    def deposited_callback(self, msg):
        self.deposited = msg.data

    def emergency_detected_callback(self, msg):
        self.emergency_detected = msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: DEPOSITING')
        
        # 状态切换
        if self.emergency_detected:
            rospy.loginfo('Emergency detected, switching to EMERGENCY')
            return 'emergency'
        elif self.deposited:
            rospy.loginfo('Deposited successfully, switching to RETURNING')
            return 'returning'
        else:
            rospy.loginfo('Continuing depositing')
            rospy.sleep(1)  # 模拟放置过程
            return 'depositing'

# 返回状态
class Returning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: RETURNING')
        rospy.sleep(2)  # 模拟返回过程
        return 'idle'

# 紧急状态
class Emergency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EMERGENCY')
        rospy.sleep(3)  # 模拟紧急处理过程
        return 'idle'

def main():
    rospy.init_node('autonomous_clamping_state_machine')

    sm = smach.StateMachine(outcomes=['END'])

    with sm:
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'approaching':'APPROACHING'})
        smach.StateMachine.add('APPROACHING', Approaching(), 
                               transitions={'grasping':'GRASPING', 
                                            'emergency':'EMERGENCY'})
        smach.StateMachine.add('GRASPING', Grasping(), 
                               transitions={'lifting':'LIFTING', 
                                            'emergency':'EMERGENCY',
                                            'grasping':'GRASPING'
                                            })
        smach.StateMachine.add('LIFTING', Lifting(), 
                               transitions={'transporting':'TRANSPORTING', 
                                            'emergency':'EMERGENCY',
                                            'lifting':'LIFTING',
                                            })
        smach.StateMachine.add('TRANSPORTING', Transporting(), 
                               transitions={'depositing':'DEPOSITING', 
                                            'emergency':'EMERGENCY',
                                            'transporting':'TRANSPORTING'
                                            })
        smach.StateMachine.add('DEPOSITING', Depositing(), 
                               transitions={'returning':'RETURNING', 
                                            'emergency':'EMERGENCY',
                                            'depositing':'DEPOSITING'
                                            },
                                            )
        smach.StateMachine.add('RETURNING', Returning(), 
                               transitions={'idle':'IDLE'})
        smach.StateMachine.add('EMERGENCY', Emergency(), 
                               transitions={'idle':'IDLE'})

    # 可视化
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()

    # 运行状态机
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()