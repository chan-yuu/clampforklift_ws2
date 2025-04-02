#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>
#include <car_interfaces/GpsImuInterface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

// tf是可以变化的

class GpsImuPublisher {
public:
    GpsImuPublisher() {
        sub_ = nh_.subscribe("/odom_p3d", 10, &GpsImuPublisher::odomCallback, this);
        pub_ = nh_.advertise<car_interfaces::GpsImuInterface>("/gps_imu", 10);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);

        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("rslidar", "base_footprint", ros::Time(0));
            car_interfaces::GpsImuInterface gps_imu;
            gps_imu.header = odom_msg->header;

            // 根据坐标变换更新位置信息
            gps_imu.x = odom_msg->pose.pose.position.x + transformStamped.transform.translation.x;
            gps_imu.y = odom_msg->pose.pose.position.y + transformStamped.transform.translation.y;
            gps_imu.z = odom_msg->pose.pose.position.z + transformStamped.transform.translation.z;

            tf2::Quaternion q;
            tf2::fromMsg(odom_msg->pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            gps_imu.roll = roll*180/M_PI;
            gps_imu.pitch = pitch*180/M_PI;
            gps_imu.yaw = yaw*180/M_PI;
            

            pub_.publish(gps_imu);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_imu_publisher");
    GpsImuPublisher gps_imu_publisher;
    ros::spin();
    return 0;
}


// 第二种，分别计算tf最后再合并

// #include <ros/ros.h>
// #include <nav_msgs/Odometry.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <geometry_msgs/TransformStamped.h>

// class LidarOdomPublisher
// {
// public:
//     LidarOdomPublisher()
//     {
//         // 初始化节点句柄
//         nh_ = ros::NodeHandle("~");

//         // 订阅odom_p3d话题
//         odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom_p3d", 10, &LidarOdomPublisher::odomCallback, this);

//         // 创建tf监听器
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

//         // 创建发布rslidar_odom话题的发布者
//         rslidar_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/rslidar_odom", 10);
//     }

//     void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
//     {
//         // 获取base_footprint到base_link的坐标变换
//         geometry_msgs::TransformStamped base_footprint_to_base_link_tf;
//         try
//         {
//             base_footprint_to_base_link_tf = tf_buffer_.lookupTransform("base_footprint", "base_link", ros::Time(0));
//         }
//         catch (tf2::TransformException &ex)
//         {
//             ROS_WARN("%s", ex.what());
//             return;
//         }

//         // 获取base_link到front_axle的坐标变换
//         geometry_msgs::TransformStamped base_link_to_front_axle_tf;
//         try
//         {
//             base_link_to_front_axle_tf = tf_buffer_.lookupTransform("base_link", "front_axle", ros::Time(0));
//         }
//         catch (tf2::TransformException &ex)
//         {
//             ROS_WARN("%s", ex.what());
//             return;
//         }

//         // 获取front_axle到rslidar_base_link的坐标变换
//         geometry_msgs::TransformStamped front_axle_to_rslidar_base_link_tf;
//         try
//         {
//             front_axle_to_rslidar_base_link_tf = tf_buffer_.lookupTransform("front_axle", "rslidar_base_link", ros::Time(0));
//         }
//         catch (tf2::TransformException &ex)
//         {
//             ROS_WARN("%s", ex.what());
//             return;
//         }

//         // 获取rslidar_base_link到rslidar的坐标变换
//         geometry_msgs::TransformStamped rslidar_base_link_to_rslidar_tf;
//         try
//         {
//             rslidar_base_link_to_rslidar_tf = tf_buffer_.lookupTransform("rslidar_base_link", "rslidar", ros::Time(0));
//         }
//         catch (tf2::TransformException &ex)
//         {
//             ROS_WARN("%s", ex.what());
//             return;
//         }

//         // 创建rslidar_odom消息
//         nav_msgs::Odometry rslidar_odom_msg;
//         rslidar_odom_msg.header = odom_msg->header;
//         rslidar_odom_msg.child_frame_id = "rslidar";

//         // 将base_footprint下的里程计信息转换到base_link坐标系下
//         geometry_msgs::PoseStamped base_footprint_pose;
//         base_footprint_pose.header = odom_msg->header;
//         base_footprint_pose.pose = odom_msg->pose.pose;
//         geometry_msgs::PoseStamped base_link_pose;
//         tf2::doTransform(base_footprint_pose, base_link_pose, base_footprint_to_base_link_tf);

//         // 将base_link下的里程计信息转换到front_axle坐标系下
//         geometry_msgs::PoseStamped front_axle_pose;
//         tf2::doTransform(base_link_pose, front_axle_pose, base_link_to_front_axle_tf);

//         // 将front_axle下的里程计信息转换到rslidar_base_link坐标系下
//         geometry_msgs::PoseStamped rslidar_base_link_pose;
//         tf2::doTransform(front_axle_pose, rslidar_base_link_pose, front_axle_to_rslidar_base_link_tf);

//         // 将rslidar_base_link下的里程计信息转换到rslidar坐标系下
//         geometry_msgs::PoseStamped rslidar_pose;
//         tf2::doTransform(rslidar_base_link_pose, rslidar_pose, rslidar_base_link_to_rslidar_tf);

//         // 设置rslidar_odom消息的pose
//         rslidar_odom_msg.pose.pose = rslidar_pose.pose;

//         // 发布rslidar_odom消息
//         rslidar_odom_pub_.publish(rslidar_odom_msg);
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber odom_sub_;
//     ros::Publisher rslidar_odom_pub_;
//     tf2_ros::Buffer tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "lidar_odom_publisher");
//     LidarOdomPublisher lidar_odom_publisher;
//     ros::spin();
//     return 0;
// }