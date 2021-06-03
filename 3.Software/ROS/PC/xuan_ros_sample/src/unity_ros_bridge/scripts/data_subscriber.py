#!/usr/bin/env python3
#coding=utf-8

import rospy
from unity_ros_bridge.msg import Xuan

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.servo_angle)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('data_subscriber_py', anonymous=True)

    rospy.Subscriber('xuan_data', Xuan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
