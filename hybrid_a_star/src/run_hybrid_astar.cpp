#include "hybrid_a_star/hybrid_a_star_flow.h"
#include "3rd/backward.hpp"
#include <ros/ros.h>

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "run_hybrid_astar");
    ros::NodeHandle node_handle;

    HybridAStarFlow kinodynamic_astar_flow(node_handle);

    // ros::Rate rate(10);
    // std::thread pathThread(&HybridAStarFlow::PublishPathAndTransform, kinodynamic_astar_flow);
    // std::thread pathThread(&HybridAStarFlow::PublishPathAndTransform, &kinodynamic_astar_flow);
    // pathThread.detach();

    ros::spin();
    // while (ros::ok()) {
    //     // std::cout<<"2"<<std::endl;
    //     // bool path_received_ = false;
    //     ros::spinOnce();
    //     // pathThread.join();
    //     kinodynamic_astar_flow.Run();

    //     rate.sleep();
    // }
    // ros::shutdown();
    return 0;
}