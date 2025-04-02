#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import rospkg
import tf.transformations  # 用于四元数转换
from geometry_msgs.msg import Pose, Quaternion


def spawn_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame):
    # 等待服务可用
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
        print("Model spawned: ", resp_sdf.success, resp_sdf.status_message)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


    
if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('spawn_model')
      # 获取包路径
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('clamp_fork')  # 替换'my_package'为你的包名
    sdf_file_path = package_path + '/urdf/cotton_ok/model.sdf'

    # 读取SDF模型文件内容
    with open(sdf_file_path, "r") as file:
        model_xml = file.read()

    # 定义模型的数量和间距
    rawNum = 2  # 行数
    lineNum = 10 # 列数
    heapNum = 3 # 堆数
    spacing = 0.45  # 每个模型之间的间距  0.45
    high = 0.5  # 高度
    dis = 2 # 相邻两堆距离
    i=0

    dis = rospy.get_param('dis', 2)  
    yaw = rospy.get_param('yaw', 0)
    lineNum = rospy.get_param('lineNum', 2)
    rawNum = rospy.get_param('rawNum', 2)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw * (3.141592653589793 / 180.0))
    # 定义模型的位置和姿态
    for h in range(heapNum):
        x=h*dis
        for r in range(rawNum):
            for l in range(lineNum-r):
                y = spacing * l + spacing*0.8 * (r)  # 错位梯字形排列
                z = high * r
                initial_pose = Pose()
                initial_pose.position.x = x  # 设置x坐标
                initial_pose.position.y = y  # 设置y坐标
                initial_pose.position.z = z  # 设置z坐标
                initial_pose.orientation.x = quaternion[0]  # 设置旋转
                initial_pose.orientation.y = quaternion[1]
                initial_pose.orientation.z = quaternion[2]
                initial_pose.orientation.w = quaternion[3]
                print(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                i+=1
                print(x,0,z,dis)
                # 可以设置四元数来定义姿态，这里默认为(0,0,0,1)即无旋转
                # 调用函数来生成模型
                spawn_model("my_robot"+str(i), model_xml, "", initial_pose, "world")

   