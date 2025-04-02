#!/usr/bin/env python
import rospy
import yaml
import numpy as np
from nav_msgs.msg import OccupancyGrid
from PIL import Image


def read_map_yaml(yaml_path):
    """
    读取map.yaml文件并解析出地图相关信息和数据
    """
    with open(yaml_path, 'r') as file:
        map_data = yaml.safe_load(file)

    resolution = map_data['resolution']
    origin = map_data['origin']
    image_path = "/home/cyun/forklift_sim_ws3/src/clamp_fork/maps/map.pgm"#map_data['image']

    # 读取.pgm图像文件获取地图数据以及宽度和高度信息
    image = Image.open(image_path)
    width, height = image.size
    grid_data = np.array(image.convert('L'))  # 转换为灰度图像并获取数据数组
    grid_data = np.flipud(grid_data) # 垂直翻转数据



    return resolution, origin, width, height, grid_data

def publish_map():
    rospy.init_node('map_publisher', anonymous=True)
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)

    yaml_path = "/home/cyun/forklift_sim_ws3/src/clamp_fork/maps/map.yaml"  # 修改为实际的map.yaml文件路径
    resolution, origin, width, height, grid_data = read_map_yaml(yaml_path)
    print("Resolution:", resolution)
    print("Origin:", origin)
    print("Width:", width)
    print("Height:", height)
    print("Grid data shape:", grid_data.shape if hasattr(grid_data, 'shape') else grid_data)

    map_msg = OccupancyGrid()
    map_msg.header.frame_id = "map"
    map_msg.header.stamp = rospy.Time.now()
    map_msg.info.resolution = resolution
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.info.origin.position.x = origin[0]
    map_msg.info.origin.position.y = origin[1]
    map_msg.info.origin.position.z = origin[2]
    map_msg.info.origin.orientation.x = 0.0
    map_msg.info.origin.orientation.y = 0.0
    map_msg.info.origin.orientation.z = 0.0
    map_msg.info.origin.orientation.w = 1.0

    # 根据阈值将地图数据转换为OccupancyGrid消息要求的格式（-1表示未知，0表示空闲，100表示占用）
    occupied_thresh = 0.65
    free_thresh = 0.196

    occupancy_data = np.where(grid_data < (255 * free_thresh), 100,
                              np.where(grid_data > (255 * occupied_thresh), 0, -1)).flatten().tolist()
    map_msg.data = occupancy_data

    rate = rospy.Rate(10)  # 1Hz的发布频率
    while not rospy.is_shutdown():
        pub.publish(map_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_map()
    except rospy.ROSInterruptException:
        pass