#!/usr/bin/env python3

import rospy
from car_interfaces.srv import PositionTask, PositionTaskRequest
#,BalesNumInCamera,BalesNumInCameraRequest

def send_position_state(data):
    rospy.loginfo("\033[91m Waiting for position_service server... \033[0m")
    rospy.wait_for_service('position_service')
    rospy.loginfo("\033[32m Position_service server ready! \033[0m")
    try:
        position_service = rospy.ServiceProxy('position_service', PositionTask)
        request = PositionTaskRequest()
        request.data = data
        response = position_service(request)
        if response.success:
            rospy.loginfo("Position state sent successfully: %s", response.message)
        else:
            rospy.loginfo("Failed to send position state: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


def send_bales_num_state(data):
    rospy.loginfo("\033[91m Waiting for bales_num_in_camera_service server... \033[0m")
    rospy.wait_for_service('bales_num_in_camera_service')
    rospy.loginfo("\033[32m bales_num_in_camera_service server ready! \033[0m")
    try:
        bales_num_in_camera_service =rospy.ServiceProxy('bales_num_in_camera_service',BalesNumInCamera)
        request = BalesNumInCameraRequest()
        request.data = data
        response = bales_num_in_camera_service(request)
        if response.success:
            rospy.loginfo("Bales num state sent successfully: %s", response.message)
        else:
            rospy.loginfo("Failed to send bales num state: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('position_state_client')
    data = 1  # 你可以根据需要修改这个值
    send_position_state(data)
    
    data = 2
    send_bales_num_state(data)