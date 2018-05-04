#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np

class InterfaceGCG():
	def __init__(self):
		self._image = np.ndarray(shape=(36, 64, 1), dtype=np.uint8)

		self._pos_x = 0
		self._pos_y = 0
		self._pos_z = 0
		self._yaw = 0
		self._pitch = 0
		self._roll = 0

		self._target_lost = False
		self._ready_for_action = False
		self._gcg_ready = True

		self._counter = 0

		rospy.init_node('interface_gcg_tb', anonymous=True)
		try:
			rospy.Subscriber('/agent/camera/camera/np_image', numpy_msg(Floats), self.callback_image)

			rospy.Subscriber('/agent/odom', Odometry, self.callback_position)

			rospy.Subscriber('/agent/odom/ypr', Vector3, self.callback_ypr)

			rospy.Subscriber('/eval', Bool, self.callback_eval)

			rospy.Subscriber('/ready_for_action', Bool, self.callback_ready)

			self._ready_pub = rospy.Publisher('/gcg_ready', Bool, queue_size=10)

			self._vel_pub = rospy.Publisher('/agent/mobile_base/commands/velocity', Twist, queue_size=10)
		except rospy.ROSInterruptException:
			pass

	# This function reads an image as an array from a topic and reshapes it back to the original shape and stores it
	def callback_image(self, data):
		image = data.data
		image = image.astype(np.uint8)
		image = image.reshape(36, 64, 1)
		self._image = image

	# This function reads out the position from a topic and stores it
	def callback_position(self, data):
		self._pos_x = data.pose.pose.position.x
		self._pos_y = data.pose.pose.position.y
		self._pos_z = data.pose.pose.position.z

	# This function reads out the orientation (ypr) from a topic and stores it
	def callback_ypr(self, data):
		self._yaw = data.x
		self._pitch = data.y
		self._roll = data.z

	# This function reads out a boolean from a topic and stores it
	def callback_eval(self, data):
		self._target_lost = data.data

	def callback_ready(self, data):
		self._ready_for_action = data.data

	# This function performs the given action if the quadrotor is ready and returns the camera image, reward, and some 
	# extra information the GCG algorithm requires
	def take_step(self, actions):
		self._counter = self._counter + 1
		
		twist = Twist()
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = actions[0][0]

		self._ready_pub.publish(self._gcg_ready)

		if (self._ready_for_action):
			self._vel_pub.publish(twist)

		if (self._target_lost):
			rewards = np.array([-1])
		else:
			rewards = np.array([0])

		if (self._counter == 1000):
			dones = np.array([True])
			self._counter = 0
		else:
			dones = np.array([self._target_lost])

		env_infos = {'pos': np.array([self._pos_x, self._pos_y, self._pos_z]), 'vel': actions[0][0], \
			'hpr': np.array([self._yaw, self._pitch, self._roll]), 'coll': self._target_lost}

		return [self._image], rewards, dones, [env_infos]
		
