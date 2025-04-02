#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <car_interfaces/GpsImuInterface.h>
#include <geometry_msgs/PoseStamped.h>

class TransformNode
{
public:
    TransformNode(ros::NodeHandle& nh)
    {
        // 从launch文件中获取map在world中的位姿
        nh.getParam("map_in_world_x", map_in_world_x_);
        nh.getParam("map_in_world_y", map_in_world_y_);
        nh.getParam("map_in_world_z", map_in_world_z_);
        nh.getParam("map_in_world_roll", map_in_world_roll_);
        nh.getParam("map_in_world_pitch", map_in_world_pitch_);
        nh.getParam("map_in_world_yaw", map_in_world_yaw_);

        nh.getParam("location_gps", location_gps);

        // 初始化 map 在 world 中的变换
        tf2::Quaternion q;
        q.setRPY(map_in_world_roll_*M_PI/180, map_in_world_pitch_*M_PI/180, map_in_world_yaw_*M_PI/180);
        map_to_world_.setOrigin(tf2::Vector3(map_in_world_x_, map_in_world_y_, map_in_world_z_));
        map_to_world_.setRotation(q);

        // 订阅 base_footprint 在 world 中的位姿
        location_sub_ = nh.subscribe("/gps_imu", 10, &TransformNode::locationCallback, this);

        // 发布 base_footprint 在 map 坐标系中的位姿
        base_in_map_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/gps_in_ndt_map", 10);
        // 代替map_pose作为车辆定位
    }

    void printYaw(const geometry_msgs::PoseStamped& pose_msg) {
        // 从PoseStamped提取四元数
        tf2::Quaternion q;
        tf2::convert(pose_msg.pose.orientation, q);

        // 计算欧拉角
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // 打印yaw角
        std::cout << "Yaw: " << yaw * (180.0 / M_PI) << " degrees" << std::endl;
    }

    void locationCallback(const car_interfaces::GpsImuInterface::ConstPtr& msg)
    {
        // std::cout<<"sub ok"<<std::endl;
        // 获取 base_footprint 在 world 坐标系中的位姿
        tf2::Transform base_to_world;
        // base_to_world.setOrigin(tf2::Vector3(msg->x, msg->y, msg->z));
        // 难道是其他参数的影响？？
        // std::cout<<"msg z: "<<msg->z<<std::endl;
        base_to_world.setOrigin(tf2::Vector3(msg->x, msg->y, msg->z));

        tf2::Quaternion q;
        // q.setRPY(msg->roll * M_PI / 180.0, msg->pitch * M_PI / 180.0, msg->yaw * M_PI / 180.0);
        // std::cout<<"msg roll: "<<msg->roll * M_PI /180<<"msg pitch: "<<msg->pitch * M_PI / 180.0<<std::endl;
        q.setRPY(msg->roll * M_PI / 180.0, msg->pitch * M_PI / 180.0, msg->yaw * M_PI / 180.0);
        base_to_world.setRotation(q);

        // 计算 base_footprint 在 map 坐标系中的位姿
        tf2::Transform world_to_map = map_to_world_.inverse();
        tf2::Transform base_to_map = world_to_map * base_to_world;

        // 发布 base_footprint 在 map 坐标系中的位姿
        geometry_msgs::PoseStamped base_in_map_msg;
        base_in_map_msg.header.stamp = ros::Time::now();
        base_in_map_msg.header.frame_id = "map";
        base_in_map_msg.pose.position.x = base_to_map.getOrigin().x();
        base_in_map_msg.pose.position.y = base_to_map.getOrigin().y();
        base_in_map_msg.pose.position.z = base_to_map.getOrigin().z();
        tf2::Quaternion base_q = base_to_map.getRotation();
        base_in_map_msg.pose.orientation = tf2::toMsg(base_q);

        base_in_map_pub_.publish(base_in_map_msg);

        // 发布 base_footprint 在 map 坐标系中的 tf
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";  // 父坐标系
        transformStamped.child_frame_id = "base_footprint";  // 子坐标系
        transformStamped.transform.translation.x = base_to_map.getOrigin().x();
        transformStamped.transform.translation.y = base_to_map.getOrigin().y();
        transformStamped.transform.translation.z = base_to_map.getOrigin().z();
        transformStamped.transform.rotation = tf2::toMsg(base_to_map.getRotation());

        // 广播 tf
        // 现在不再使用ndt进行定位，所以通过这个方式来发布我需要的定位tf
        if (location_gps){
            tf_broadcaster_.sendTransform(transformStamped);
        }
        else{
            ROS_INFO_ONCE("location_gps is false");
        }
        
    }

private:
    ros::Subscriber location_sub_;
    ros::Publisher base_in_map_pub_;
    bool location_gps;

    // tf 广播器
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // map 相对于 world 的位姿
    double map_in_world_x_, map_in_world_y_, map_in_world_z_;
    double map_in_world_roll_, map_in_world_pitch_, map_in_world_yaw_;

    // 存储 map 在 world 中的变换
    tf2::Transform map_to_world_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_node");
    ros::NodeHandle nh;

    // 创建 TransformNode 实例
    TransformNode transform_node(nh);

    ros::spin();
    return 0;
}
