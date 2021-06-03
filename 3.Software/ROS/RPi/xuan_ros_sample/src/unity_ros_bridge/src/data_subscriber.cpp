#include "ros/ros.h"
#include "std_msgs/String.h"
#include "unity_ros_bridge/Xuan.h"

void chatterCallback(const unity_ros_bridge::Xuan::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->servo_angle);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "xuan_subscriber_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("xuan_data", 100, chatterCallback);

  ros::spin();


  return 0;
}

