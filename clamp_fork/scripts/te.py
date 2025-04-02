#!/usr/bin/env python

import rospy
import sys
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_sdf_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame="world"):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp = spawn_model_prox(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF model service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('spawn_sdf_model_node')

    # Model name
    model_name = "my_model"

    # Read the SDF file
    with open("/home/cyun/forklift_sim_ws/src/clamp_fork/urdf/cotton_ok/model.sdf", "r") as file:
        model_xml = file.read()

    # Namespace for the robot
    robot_namespace = "/"

    # Initial pose of the model
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    initial_pose.position.z = 0
    initial_pose.orientation.x = 0
    initial_pose.orientation.y = 0
    initial_pose.orientation.z = 0
    initial_pose.orientation.w = 1

    # Spawn the model
    spawn_sdf_model(model_name, model_xml, robot_namespace, initial_pose)