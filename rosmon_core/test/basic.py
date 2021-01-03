#!/usr/bin/env python

# Test driver for basic test

from __future__ import print_function

import sys
import unittest
import time
import os
import math

import rospy
import rospy.client

import rospkg

from std_msgs.msg import String

from rosmon_msgs.msg import State, NodeState

rospack = rospkg.RosPack()

class _WFM(object):
	def __init__(self):
		self.msg = None
	def cb(self, msg):
		if self.msg is None:
			self.msg = msg

## A sample python unit test
class BasicTest(unittest.TestCase):

	def get_param(self, name):
		try:
			return rospy.get_param(name)
		except KeyError as e:
			params = ', '.join([ '"' + name + '"' for name in sorted(rospy.get_param_names()) ])
			raise KeyError(
				'Caught KeyError("%s") while getting parameter. Known parameters: %s' % (e, params)
			)

	def test_rosmon_running(self):
		try:
			state = rospy.client.wait_for_message('/rosmon_uut/state', State, timeout=5.0)
		except rospy.ROSException:
			self.fail('Did not get state msg on /rosmon_uut/state' + repr(rospy.client.get_published_topics()))

		self.assertEqual(len(state.nodes), 4)

		test1 = [ n for n in state.nodes if n.name == 'test1' ]
		self.assertEqual(len(test1), 1)
		self.assertEqual(test1[0].state, NodeState.RUNNING)

	def test_remapping(self):
		pub = rospy.Publisher('/test_input', String, queue_size=5, latch=True)

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
		self.assertEqual(self.get_param("path_to_rosmon"), rospack.get_path('rosmon_core'))
		self.assertEqual(
			self.get_param("path_to_launch_file"),
			os.path.join(rospack.get_path('rosmon_core'), 'test/basic.launch')
		)
		executable = self.get_param("path_to_rosmon_executable")
		self.assert_(os.access(executable, os.X_OK), 'Invalid rosmon path: ' + executable)

		self.assertEqual(
			self.get_param("dirname"),
			os.path.join(rospack.get_path('rosmon_core'), 'test')
		)

		self.assertEqual(self.get_param("eval_simple"), True)
		self.assertEqual(self.get_param("eval_argexpr"), True)
		self.assertAlmostEqual(self.get_param("eval_radius_pi"), 0.5 * math.pi)

		self.assertEqual(self.get_param("/test1/private_param1"), "hello_world")
		self.assertEqual(self.get_param("/test1/private_param2"), "hello_world")

	def test_multiLine(self):
		self.assertEqual(self.get_param("multiple_lines1"), "first_line  second_line")
		self.assertEqual(self.get_param("multiple_lines2"), "first_line second_line")

	def test_yaml(self):
		self.assertAlmostEqual(self.get_param("yaml/radius"), 0.5)

	def test_arg_passing(self):
		self.assertEqual(self.get_param("test_argument"), 123)

	def test_global_remapping(self):
		pub = rospy.Publisher('/remapped_test_input', String, queue_size=5, latch=True)

		wfm = _WFM()
		sub = rospy.Subscriber('/remapped_test_output', String, wfm.cb)

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

	def test_nested(self):
		self.assertEqual(rospy.get_param("/nested/nested_param"), "hello")

if __name__ == '__main__':
	rospy.init_node('basic_test')

	# Wait for rosmon to start up
	state = rospy.client.wait_for_message('/rosmon_uut/state', State, timeout=5.0)

	import rostest
	rostest.rosrun('rosmon_core', 'basic', BasicTest)

	time.sleep(1)
