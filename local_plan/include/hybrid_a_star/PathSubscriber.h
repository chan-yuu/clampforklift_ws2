#ifndef PATH_SUBSCRIBER_H
#define PATH_SUBSCRIBER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>

class PathSubscriber {
public:
    PathSubscriber(ros::NodeHandle& nh, const std::string& topic_name, uint32_t queue_size) {
        subscriber_ = nh.subscribe(topic_name, queue_size, &PathSubscriber::callback, this);
    }

    // 获取最新接收到的路径
    nav_msgs::Path getLatestPath() const {
        return latest_path_;
    }

private:
    void callback(const nav_msgs::Path::ConstPtr& msg) {
        latest_path_ = *msg; // 存储最新的路径
        // 在这里处理路径，或者通过getLatestPath()在外部处理
    }

    ros::Subscriber subscriber_;
    nav_msgs::Path latest_path_; // 存储最新接收到的路径
};

#endif // PATH_SUBSCRIBER_H
