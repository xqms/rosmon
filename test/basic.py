#!/usr/bin/env python

# Test driver for basic test

import sys
import unittest
import time

import rospy
import rospy.client

from std_msgs.msg import String

class _WFM(object):
	def __init__(self):
		self.msg = None
	def cb(self, msg):
		if self.msg is None:
			self.msg = msg

## A sample python unit test
class BasicTest(unittest.TestCase):

	def test_rosmon_running(self):
		from rosmon.msg import State, NodeState

		try:
			state = rospy.client.wait_for_message('/rosmon_uut/state', State, timeout=5.0)
		except rospy.ROSException:
			self.fail('Did not get state msg on /rosmon_uut/state' + repr(rospy.client.get_published_topics()))

		self.assertEqual(len(state.nodes), 1)
		self.assertEqual(state.nodes[0].name, 'test1')
		self.assertEqual(state.nodes[0].state, NodeState.RUNNING)

	def test_remapping(self):
		pub = rospy.Publisher('/test_input', String, queue_size=5)

		wfm = _WFM()
		sub = rospy.Subscriber('/test_output', String, wfm.cb)

		time.sleep(1);

		self.assertGreater(pub.get_num_connections(), 0)
		self.assertGreater(sub.get_num_connections(), 0)
		pub.publish('Hello world!')

		timeout_t = time.time() + 5
		while wfm.msg is None:
			rospy.rostime.wallsleep(0.01)
			if time.time() >= timeout_t:
				self.fail('No reply to test message')

		self.assertEqual(wfm.msg.data, 'Hello world!')

		sub.unregister()
		pub.unregister()

if __name__ == '__main__':
	rospy.init_node('basic_test')

	import rostest
	rostest.rosrun('rosmon', 'basic', BasicTest)
