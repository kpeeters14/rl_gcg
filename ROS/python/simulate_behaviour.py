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

target_yaw = 0
target_yaw_min = 0
target_yaw_max = 0

target_lost = False
gcg_ready = False

# # This function is a FSM that tells the drone what to do in each state and returns a twist (velocity) message
# def fsm_datapath():
#   twist = Twist()

#   # INITIALIZE
#   # Wait until everything is ready
#   if (state == 0):
#     twist.linear.x = 0.0
#     twist.linear.y = 0.0
#     twist.linear.z = 0.0
#     twist.angular.x = 0.0
#     twist.angular.y = 0.0
#     twist.angular.z = 0.0

#   # TAKE OFF  
#   # Let the drone take off (give it a speed along the positive z-axis)
#   elif (state == 1):
#     twist.linear.x = 0.0
#     twist.linear.y = 0.0
#     twist.linear.z = 0.25
#     twist.angular.x = 0.0
#     twist.angular.y = 0.0
#     twist.angular.z = 0.0

#   # END TAKE OFF 
#   # Let the drone levitate at its current position (give it zero speed along the positive z-axis)
#   elif (state == 2):
#     twist.linear.x = 0.0
#     twist.linear.y = 0.0
#     twist.linear.z = 0.0
#     twist.angular.x = 0.0
#     twist.angular.y = 0.0
#     twist.angular.z = 0.0

#   # RESET ORIENTATION
#   # Reset the orientation of the drone so that the target object is in sight again (give it a speed around the z-axis)
#   elif (state == 4):
#     twist.linear.x = 0.0
#     twist.linear.y = 0.0
#     twist.linear.z = 0.0
#     twist.angular.x = 0.0
#     twist.angular.y = 0.0
#     if ((yaw < 3.15) and (target_yaw > -3.15)):
#       twist.angular.z = 0.25
#     elif ((target_yaw < 3.15) and (yaw  > -3.15)):
#       twist.angular.z = -0.25
#     else:
#       if (yaw < target_yaw_min):
#         twist.angular.z = 0.25
#       elif (yaw > target_yaw_max):
#         twist.angular.z = -0.25
#       else:
#         twist.angular.z = 0

#   # LAND
#   # Let the drone land (give it a speed along the negative z-axis)
#   elif (state == 5):
#     twist.linear.x = 0.0
#     twist.linear.y = 0.0
#     twist.linear.z = -0.25
#     twist.angular.x = 0.0
#     twist.angular.y = 0.0
#     twist.angular.z = 0.0

#   return twist

# # This function is a FSM that takes care of the state transitions for the drone and returns the state.
# def fsm_controller():
#   global state

#   # INITIALIZE
#   # Wait until everything is ready
#   if (state == 0)=
#       if (gcg_ready):
#       print("Taking off...")
#       state = 1
#     else:
#       state = 0

#   # TAKE OFF  
#   # Let the drone take off (give it a speed along the positive z-axis)
#   elif (state == 1):
#     if (pos_z > 0.25):
#       print("Take off succesful!")
#       state = 2
#     else:
#       state = 1

#   # END TAKE OFF 
#   # Let the drone levitate at its current position (give it zero speed along the positive z-axis)
#   elif (state == 2):
#     print("Following target object...")
#     state = 3

#   # FOLLOW TARGET
#   # Make the drone follow the target object (give it a speed around the z-axis)
#   elif (state == 3):
#     if (target_lost):
#       print("Target object was lost. Resetting orientation...")
#       state = 4
#     else:
#       state = 3

#   # RESET ORIENTATION
#   # Reset the orientation of the drone so that the target object is in sight again (give it a speed around the z-axis)
#   elif (state == 4):
#     if (abs(target_yaw - yaw) < 0.05):
#       print("Orientation was reset. Landing...")
#       state = 5
#     else:
#       state = 4
      
#   # LAND
#   # Let the drone land (give it a speed along the negative z-axis)
#   elif (state == 5):
#     if ((pos_z > -0.05) and (pos_z < 0.05)):
#       print("Initializing everything. Please wait...")
#       state = 0
#     else:
#       state = 5

#   return state

# This function is a FSM that tells the drone what to do in each state and returns a twist (velocity) message
def fsm_datapath():
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

  # TAKE OFF  
  # Let the drone take off (give it a speed along the positive z-axis)
  elif (state == 1):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.25
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

  # END TAKE OFF 
  # Let the drone levitate at its current position (give it zero speed along the positive z-axis)
  elif (state == 2):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

  # RESET ORIENTATION
  # Reset the orientation of the drone so that the target object is in sight again (give it a speed around the z-axis)
  elif (state == 4):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    if ((yaw < 3.15) and (target_yaw > -3.15)):
      twist.angular.z = 0.25
    elif ((target_yaw < 3.15) and (yaw  > -3.15)):
      twist.angular.z = -0.25
    else:
      if (yaw < target_yaw_min):
        twist.angular.z = 0.25
      elif (yaw > target_yaw_max):
        twist.angular.z = -0.25
      else:
        twist.angular.z = 0

  # RESET POSITION
  # Reset the position of the drone so that it is more or less in the origin again
  elif (state == 5):
    if (pos_x > 0.15):
      twist.linear.x = -0.05
    elif (pos_x < -0.15):
      twist.linear.x = 0.05
    else:
      twist.linear.x = 0.0
      if (pos_y > 0.15):
        twist.linear.y = -0.05
      elif (pos_y < -0.15):
        twist.linear.y = 0.05
      else:
        twist.linear.y = 0.0
    twist.linear.z = 0.0

  return twist

# This function is a FSM that takes care of the state transitions for the drone and returns the state.
def fsm_controller():
  global state

  # INITIALIZE
  # Wait until everything is ready
  if (state == 0):
    # if (gcg_ready):
    if (True):
      print("Taking off...")
      state = 1
    else:
      state = 0

  # TAKE OFF  
  # Let the drone take off (give it a speed along the positive z-axis)
  elif (state == 1):
    if (pos_z > 0.25):
      print("Take off succesful!")
      state = 2
    else:
      state = 1

  # END TAKE OFF 
  # Let the drone levitate at its current position (give it zero speed along the positive z-axis)
  elif (state == 2):
    print("Following target object...")
    state = 3

  # FOLLOW TARGET
  # Make the drone follow the target object (give it a speed around the z-axis)
  elif (state == 3):
    if (target_lost):
      print("Target object was lost. Resetting orientation...")
      state = 4
    else:
      state = 3

  # RESET ORIENTATION
  # Reset the orientation of the drone so that the target object is in sight again (give it a speed around the z-axis)
  elif (state == 4):
    if (abs(target_yaw - yaw) < 0.05):
      print("Orientation was reset.")
      state = 5
    else:
      state = 4

  # RESET POSITION
  # Reset the position of the drone so that it is more or less in the origin again
  elif (state == 5):
    if ((abs(pos_x) < 0.15) and (abs(pos_y) < 0.15)):
      print("Position was reset.")
      state = 2
    else:
      state = 5

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

  if (state == 3):
    ready_for_action = True
  else:
    ready_for_action = False
    vel_pub.publish(twist)

  ready_pub.publish(ready_for_action)

# This function reads out the position from a topic and transforms the orientation (in quaternions) to a ypr representation
def callback_position(data):
  global pos_x, pos_y, pos_z, yaw, pitch, roll

  pos_x = data.pose.pose.position.x
  pos_y = data.pose.pose.position.y
  pos_z = data.pose.pose.position.z

  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, \
    data.pose.pose.orientation.z, data.pose.pose.orientation.w))

# This function reads out the target position from a topic and calculates the yaw range where the agent should be in when
# resetting orientation
def callback_target_position(data):
  global target_yaw, target_yaw_min, target_yaw_max

  target_pos_x = 3 - (data.pose.pose.position.y - pos_x)  # Axes from target are not the same as those from agent 
  target_pos_y = data.pose.pose.position.x                # --> conversion to express target coordinates relative to agent axes

  target_yaw = np.arctan(max(abs(target_pos_y), 0.001)/max(abs(target_pos_x), 0.001))
  if ((target_pos_x < 0) and (target_pos_y > 0)):
    target_yaw = 3.14 - target_yaw
  elif ((target_pos_x < 0) and (target_pos_y < 0)):
    target_yaw = -3.14 + target_yaw
  elif ((target_pos_x > 0) and (target_pos_y < 0)):
    target_yaw = -target_yaw

  target_yaw_min = target_yaw - 0.05
  target_yaw_max = target_yaw + 0.05

# This function reads out a boolean from a topic and stores it globally in target_lost
def callback_eval(data):
  global target_lost
  target_lost = data.data

# This function reads out a boolean from a topic and stores it globally in gcg_ready
def callback_gcg_ready(data):
  global gcg_ready
  gcg_ready = data.data


if __name__=="__main__":
  rospy.init_node('simulate_behaviour', anonymous=True)
  try:
    rospy.Subscriber('/agent/kinect/kinect/rgb/image_raw', Image, callback_image)

    vel_pub = rospy.Publisher('/agent/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/agent/ground_truth/state', Odometry, callback_position)

    rospy.Subscriber('/target/odom', Odometry, callback_target_position)

    rospy.Subscriber('/eval', Bool, callback_eval)

    rospy.Subscriber('/gcg_ready', Bool, callback_gcg_ready)

    ready_pub = rospy.Publisher('/ready_for_action', Bool, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
