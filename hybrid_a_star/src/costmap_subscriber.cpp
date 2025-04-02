#include "hybrid_a_star/costmap_subscriber.h"
// CostMapSubscriber类的构造函数
// 输入参数：
//   - nh：ROS节点句柄，用于与ROS系统交互
//   - topic_name：要订阅的主题名称，这里应该是发布栅格地图(OccupancyGrid)的主题
//   - buff_size：订阅消息队列的大小
CostMapSubscriber::CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(topic_name, buff_size, &CostMapSubscriber::MessageCallBack, this);
}

void CostMapSubscriber::MessageCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr) {
    buff_mutex_.lock();
    deque_costmap_.emplace_back(costmap_msg_ptr);
    buff_mutex_.unlock();
}

// 解析数据函数
// 该函数的目的是将接收到的栅格地图消息从内部队列(deque_costmap_)转移到外部队列(deque_costmap_msg_ptr)
// 输入参数：
//   - deque_costmap_msg_ptr：外部的栅格地图消息指针队列，用于接收内部队列的数据
void CostMapSubscriber::ParseData(std::deque<nav_msgs::OccupancyGridPtr> &deque_costmap_msg_ptr) {
    buff_mutex_.lock();
    if (!deque_costmap_.empty()) {
        deque_costmap_msg_ptr.insert(deque_costmap_msg_ptr.end(),
                                     deque_costmap_.begin(),
                                     deque_costmap_.end()
        );

        deque_costmap_.clear();
    }
    buff_mutex_.unlock();
}