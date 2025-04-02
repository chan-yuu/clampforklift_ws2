// main.cpp
#include "ros/ros.h"
#include "control/can_feedback.h"
#include <thread>


void Can(ros::NodeHandle &nodehandle,CanFeedBack &canFeedBack) {
    while (ros::ok()) {
        canFeedBack.DecodeCanData();
        // rate.sleep();
    }
    close(canFeedBack.s);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    
    CanFeedBack canFeedBack(nh);

    std::thread t1(Can, std::ref(nh),std::ref(canFeedBack));
    // std::thread t2(Control, std::ref(nh),std::ref(canFeedBack));
    // t2.join();
    t1.join();  

    std::cout << "Thread has finished.\n";
    return 0;
}
