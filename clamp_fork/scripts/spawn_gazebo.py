#!/usr/bin/env python

import rospy
import rospkg
import random
import time
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty

def load_xacro(xacro_file_path, parameters=None):
    import subprocess
    cmd = ["xacro", xacro_file_path]
    # if parameters:
    #     for key, value in parameters.items():
    #         cmd.append(f"{key}:={value}")
    
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode != 0:
        raise Exception("Failed to parse xacro file: " + result.stderr.decode())
    return result.stdout.decode()

from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState


def check_model_exists(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_model_state(model_name, 'world')
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

def spawn_model(model_name, model_xml, initial_pose, reference_frame='world'):
    print(333333)
    # rospy.wait_for_service('gazebo/spawn_urdf_model')
    rospy.wait_for_service('/gazebo/delete_model', timeout=10.0)
    # print(444444)
    try:
        print(5555)
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        resp = spawn_model_prox(model_name, model_xml, "", initial_pose, reference_frame)
        print(f"Spawned model {model_name}")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

def reset():
    # 等待 reset_world 服务变得可用
    rospy.wait_for_service('/gazebo/reset_world')
    
    try:
        # 创建服务代理
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
        # 调用服务
        reset_world()
        print("Gazebo world has been reset.")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def delete_model(model_name):
    rospy.wait_for_service('gazebo/delete_model')
    try:
        delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        print(66666)
        time.sleep(2)
        resp = delete_model_prox(model_name)
        print(777777)
        time.sleep(5)
        print(f"Deleted model {model_name}")
        return resp.success
        return True
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

def main():
    rospy.init_node('spawn_random_models')

    # 模型的名称和路径
    model_name = 'smart'
    package_path = rospkg.RosPack().get_path('clamp_fork')
    xacro_file_path = package_path + '/urdf/fork.xacro'

    # # 定义传递给xacro的参数
    # parameters = {
    #     'roboname': model_name  # 或者其他你所需要的值
    # }

    try:
        print("Loading xacro file from path:", xacro_file_path)
        model_xml = load_xacro(xacro_file_path)

        # 设置 robot_description 参数
        rospy.set_param('robot_description', model_xml)

        while not rospy.is_shutdown():
            # 随机生成位置和姿态
            initial_pose = Pose(
                position=Point(
                    x=random.uniform(-5, 5),
                    y=random.uniform(-5, 5),
                    z=0  # 假设模型的高度为0.5m
                ),
                orientation=Quaternion(
                    x=0,
                    y=0,
                    z=random.uniform(-1, 1),
                    w=random.uniform(-1, 1)
                )
            )
            print(111)



            # 如果模型存在，则先删除旧模型
            if check_model_exists(model_name):
                if not delete_model(model_name):
                    print("Failed to delete existing model, retrying...")
                    continue

                # 等待一段时间确保模型删除成功
                time.sleep(2)

            # 放置模型到随机位置
            if not spawn_model(model_name, model_xml, initial_pose):
                print(222)

                print("Failed to spawn model, retrying...")
                # continue
            print(333)

            # 等待1秒
            # time.sleep(1)
            # 删除已有的模型
            # delete_model(model_name)
        # reset()
        # if delete_model(model_name):
        #     print("Model deleted successfully.")
        # else:
        #     print("Failed to delete model.")
    except Exception as e:
        print(e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass