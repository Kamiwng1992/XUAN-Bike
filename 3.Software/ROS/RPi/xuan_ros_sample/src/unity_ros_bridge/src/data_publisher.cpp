#include "ros/ros.h"
#include "unity_ros_bridge/Xuan.h"
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "xuan_publisher_node");

  ros::NodeHandle n;
 
  ros::Publisher chatter_pub = n.advertise<unity_ros_bridge::Xuan>("xuan_data", 100);
 
  ros::Rate loop_rate(1);
 
  int count = 0;
  float f = 1;
  while (ros::ok())
  {
 
    unity_ros_bridge::Xuan msg;
    msg.servo_angle = 222;

    chatter_pub.publish(msg);

 
    loop_rate.sleep();
 
     ROS_INFO("%d", count);
  }


  return 0;
}
 
