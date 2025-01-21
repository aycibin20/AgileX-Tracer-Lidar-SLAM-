#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(data):
    rospy.loginfo("Position: x=%f, y=%f, z=%f", 
                  data.pose.pose.position.x, 
                  data.pose.pose.position.y, 
                  data.pose.pose.position.z)

def listener():
    rospy.init_node('ground_truth_listener', anonymous=True)
    rospy.Subscriber('/ground_truth/state', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

