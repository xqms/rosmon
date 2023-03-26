#!/usr/bin/env python

from __future__ import print_function

import rospy
from std_msgs.msg import String


rospy.init_node('test_node')

pub = rospy.Publisher('~output', String, queue_size=10)

def callback(data):
	rospy.loginfo("Test node got {}".format(data))
	pub.publish(data.data)

sub = rospy.Subscriber('~input', String, callback)

rospy.spin()
