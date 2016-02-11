#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String

def callback(data):
	print rospy.get_caller_id() + " DATA: " + data.data

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("caminfo", String, callback)
	rospy.spin()

if __name__ == "__main__":
	listener()
