#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import time
import tf
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()

state = 0

pos_x = 0
pos_y = 0
pos_z = 0
yaw = 0
pitch = 0
roll = 0

inner_wall_distance = 0
outer_wall_distance = 0

center_yaw = 0
target_yaw = 0

collision = False
gcg_ready = False

# This function is a FSM that tells the agent what to do in each state and returns a twist (velocity) message
def fsm_datapath():
  global target_yaw
  twist = Twist()

  # INITIALIZE
  # Wait until everything is ready
  if (state == 0):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

  # FACE CENTER
  # Make the agent face opposite to the center of the room
  if (state == 2):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    if (abs(yaw - center_yaw) > 0.02):
      twist.angular.z = 0.25
    else:
      twist.angular.z = 0

  # RESET POSITION
  # Reset the position of the agent until it is far enough from the closest wall
  elif (state == 3):
    if (inner_wall_distance < 2.45):
      twist.linear.x = 0.5
    elif (outer_wall_distance < 2.45):
      twist.linear.x = -0.5
    else:
      twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

  # RESET ORIENTATION
  # Make the agent face the correct orientation again in order to proceed through the corridor
  if (state == 4):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    if ((pos_y >= 5) and ((pos_x <= 5) and (pos_x >= -10))):
      target_yaw = 0.0
    elif ((pos_x >= 5) and ((pos_y <= 10) and (pos_y >= -5))):
      target_yaw = -1.57
    elif ((pos_y <= -5) and ((pos_x <= 10) and (pos_x >= -5))):
      target_yaw = 3.14
    else:
      target_yaw = 1.57
    if (abs(yaw - target_yaw) > 0.02):
      twist.angular.z = -0.25
    else:
      twist.angular.z = 0

  return twist

# This function is a FSM that takes care of the state transitions for the agent and returns the state.
def fsm_controller():
  global state

  # INITIALIZE
  # Wait until everything is ready
  if (state == 0):
    if (gcg_ready):
      print("Driving...")
      state = 1
    else:
      state = 0

  # DRIVE
  # Drive around with a constant linear speed and changing angular speed
  elif (state == 1):
    if (collision):
      print("A collision happened. Facing away from center...")
      state = 2
    else:
      state = 1

  # FACE CENTER
  # Make the agent face opposite to the center of the room
  if (state == 2):
    if (abs(yaw - center_yaw) < 0.02):
      print("Resetting position...")
      state = 3
    else:
      state = 2

  # RESET POSITION
  # Reset the position of the agent until it is far enough from the closest wall
  elif (state == 3):
    if ((inner_wall_distance > 2.45) and (outer_wall_distance > 2.45)):
      print("Position reset. Resetting orientation...")
      state = 4
    else:
      state = 3

  # RESET ORIENTATION
  # Make the agent face the correct orientation again in order to proceed through the corridor
  if (state == 4):
    if (abs(yaw - target_yaw) < 0.02):
      print("Orientation reset.")
      state = 0
    else:
      state = 4

  return state

# This function reads an image from a topic, transforms it into a numpy array, shows the image on screen, runs the FSM
# functions and finally publishes the twist (velocity) on another topic if ready. The ready state is also published on a topic
def callback_image(data):
  try:
    # Convert your ROS Image message to OpenCV2
    image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError, e:
    print(e)
  else:
    cv2.imshow('Control',image)
    cv2.waitKey(2)

  state = fsm_controller()
  twist = fsm_datapath()

  if (state == 1):
    ready_for_action = True
  else:
    ready_for_action = False
    vel_pub.publish(twist)

  ready_pub.publish(ready_for_action)

# This function reads out the position from a topic and transforms the orientation (in quaternions) to a ypr representation
# It also calculates the yaw looking at the center of the room and the distance to the walls.
def callback_position(data):
  global pos_x, pos_y, pos_z, yaw, pitch, roll, center_yaw, inner_wall_distance, outer_wall_distance

  pos_x = data.pose.pose.position.x
  pos_y = data.pose.pose.position.y + 7.5 # Express coordinates relative to the center of the room (0.0, -7.5, 0.0)
  pos_z = data.pose.pose.position.z

  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, \
    data.pose.pose.orientation.z, data.pose.pose.orientation.w))

  center_yaw = np.arctan(max(abs(pos_y), 0.001)/max(abs(pos_x), 0.001))
  if ((pos_x < 0) and (pos_y > 0)):
    center_yaw = 3.14 - center_yaw
  elif ((pos_x < 0) and (pos_y < 0)):
    center_yaw = -3.14 + center_yaw
  elif ((pos_x > 0) and (pos_y < 0)):
    center_yaw = -center_yaw

  outer_wall_distance = min((10 - abs(pos_y)),(10 - abs(pos_x)))
  inner_wall_distance = 5 - outer_wall_distance

# This function reads out a boolean from a topic and stores it globally in collision
def callback_collision(data):
  global collision
  collision = data.data

# This function reads out a boolean from a topic and stores it globally in gcg_ready
def callback_gcg_ready(data):
  global gcg_ready
  gcg_ready = data.data

if __name__=="__main__":
  rospy.init_node('behaviour_tb_ca', anonymous=True)
  try:
    rospy.Subscriber('/camera/camera/rgb/image_raw', Image, callback_image)

    vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    rospy.Subscriber('/odom', Odometry, callback_position)

    rospy.Subscriber('/collision', Bool, callback_collision)

    rospy.Subscriber('/gcg_ready', Bool, callback_gcg_ready)

    ready_pub = rospy.Publisher('/ready_for_action', Bool, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin()
  except rospy.ROSInterruptException:
        pass
