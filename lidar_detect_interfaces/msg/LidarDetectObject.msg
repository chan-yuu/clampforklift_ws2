# LidarDetectObject.msg

Header header

# Object metadata
string label                    # 对象标签
float64 score                   # 置信度评分
string space_frame              # 空间参考帧

# Pose and dimensions
geometry_msgs/Pose pose         # 对象中心位置和姿态
geometry_msgs/Vector3 dimensions # 对象尺寸 (长宽高)

# Point cloud data
sensor_msgs/PointCloud2 pointcloud       # 对象点云数据
geometry_msgs/PolygonStamped convex_hull # 凸包表示

# Validity flag
bool valid                      # 是否有效

# Bounding box information
float64 length                  # 长度
float64 width                   # 宽度
float64 height                  # 高度
float64 core_x                  # 中心点 X 坐标
float64 core_y                  # 中心点 Y 坐标
float64 core_z                  # 中心点 Z 坐标
float64 yaw                     # 偏航角

# Additional properties
int32 id                        # 对象 ID
float64 speed                   # 速度
int32 type                      # 类型
int32 exist_con                 # 存在性置信度
float64 acc_x                   # 加速度 X 分量
float64 acc_y                   # 加速度 Y 分量
float64 acc_z                   # 加速度 Z 分量

# Time stamp
int64 timeStamps                # 时间戳 (纳秒)