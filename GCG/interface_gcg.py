#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np

class InterfaceGCG(object):
	def __init__(self):
		self._image = np.ndarray(shape=(36,64), dtype=np.uint8)

		self._pos_x = 0
		self._pos_y = 0
		self._pos_z = 0
		self._yaw = 0
		self._pitch = 0
		self._roll = 0

		self._target_lost = False
		self._ready = False

		rospy.init_node('interface_gcg', anonymous=True)
		try:
			rospy.Subscriber('/kinect/kinect/np_image', numpy_msg(Floats), self.callback_image)

			rospy.Subscriber('/ground_truth/state', Odometry, self.callback_position)

			rospy.Subscriber('/ground_truth/ypr', Vector3, self.callback_ypr)

			rospy.Subscriber('/eval', Bool, self.callback_eval)

			rospy.Subscriber('/ready', Bool, self.callback_ready)

			self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

			# spin() simply keeps python from exiting until this node is stopped  
			rospy.spin()
		except rospy.ROSInterruptException:
			pass

	# This function reads an image as an array from a topic and reshapes it back to the original shape and stores it
	def callback_image(self, data):
		image = data.data
		image = image.astype(np.uint8)
		image = image.reshape(36, 64)
		self._image = image

	# This function reads out the position from a topic and stores it
	def callback_position(self, data):
		self._pos_x = data.pose.pose.position.x
		self._pos_y = data.pose.pose.position.y
		self._pos_z = data.pose.pose.position.z

	# This function reads out the orientation (ypr) from a topic and stores it
	def callback_position(self, data):
		self._yaw = data.x
		self._pitch = data.y
		self._roll = data.z

	# This function reads out a boolean from a topic and stores it
	def callback_eval(self, data):
		self._target_lost = data.data

	def callback_ready(self, data):
		self._ready = data.data

	# This function performs the given action if the quadrotor is ready and returns the camera image, reward, and some 
	# extra information the GCG algorithm requires
	def take_step(self, actions):
		twist = Twist()
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = actions[0]

		if (self._ready):
			self._vel_pub.publish(twist)

		if (self._target_lost):
			rewards = -1
		else:
			rewards = 0

		dones = self._target_lost

		env_infos = {'pos': (self._pos_x, self._pos_y, self._pos_z), 'vel': actions[0], \
			'hpr': (self._yaw, self._pitch, self._roll), 'col': self._target_lost}

		return [self._image, rewards, dones, env_infos]
		
