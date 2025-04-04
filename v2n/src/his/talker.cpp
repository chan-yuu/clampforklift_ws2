#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
int a;
int demo(){
    std::cout<<a;
    return 0;
}

int main(int argc, char **argv)
{
    a = 1;
  ros::init(argc, argv, "talker");


  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
demo();
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << a;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}