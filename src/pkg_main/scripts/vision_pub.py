#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String

def publish_info_repeat():
	pub = rospy.Publisher('caminfo', String, queue_size=2)
	rospy.init_node('vision_pub', anonymous=False)
	rate = rospy.Rate(10) # Hz
	while not rospy.is_shutdown():
		pub.publish("STUB")
		rate.sleep()
	
if __name__ == "__main__":
	try:
		publish_info_repeat()
	except rospy.ROSInterruptException:
		pass
