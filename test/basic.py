#!/usr/bin/env python

# Test driver for basic test

import sys
import unittest
import time
import os
import math

import rospy
import rospy.client

import rospkg

from std_msgs.msg import String

rospack = rospkg.RosPack()

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

	def test_params(self):
		self.assertEqual(rospy.get_param("path_to_rosmon"), rospack.get_path('rosmon'))
		self.assertEqual(
			rospy.get_param("path_to_launch_file"),
			os.path.join(rospack.get_path('rosmon'), 'test/basic.launch')
		)
		executable = rospy.get_param("path_to_rosmon_executable")
		self.assert_(os.access(executable, os.X_OK), 'Invalid rosmon path: ' + executable)

		self.assertEqual(
			rospy.get_param("dirname"),
			os.path.join(rospack.get_path('rosmon'), 'test')
		)

		self.assertEqual(rospy.get_param("eval_simple"), True)
		self.assertEqual(rospy.get_param("eval_argexpr"), True)
		self.assertAlmostEqual(rospy.get_param("eval_radius_pi"), 0.5 * math.pi)

if __name__ == '__main__':
	rospy.init_node('basic_test')

	import rostest
	rostest.rosrun('rosmon', 'basic', BasicTest)
