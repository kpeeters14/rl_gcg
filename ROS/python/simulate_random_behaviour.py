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
import random
import tf

# Instantiate CvBridge
bridge = CvBridge()

state = 0

pos_x = 0
pos_y = 0
pos_z = 0
yaw = 0
pitch = 0
roll = 0

target_lost = False

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

  # FOLLOW TARGET
  # Make the drone follow the target object (give it a speed around the z-axis)
  elif (state == 3):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = random.uniform(-1, 1)

  # RESET ORIENTATION
  # Reset the orientation of the drone so that the target object is in sight again (give it a speed around the z-axis)
  elif (state == 4):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    if (yaw < 1.62):
      twist.angular.z = 0.25
    elif (yaw > 1.52):
      twist.angular.z = -0.25
    else:
      twist.angular.z = 0

  # LAND
  # Let the drone land (give it a speed along the negative z-axis)
  elif (state == 5):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = -0.25
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

  return twist

# This function is a FSM that takes care of the state transitions for the drone and returns the state.
def fsm_controller():
  global state

  # INITIALIZE
  # Wait until everything is ready
  if (state == 0):
    time.sleep(5)
    print("Taking off...")
    state = 1

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
    if ((yaw < 1.62) and (yaw > 1.52)):
      print("Orientation was reset. Landing...")
      state = 5
    else:
      state = 4
      
  # LAND
  # Let the drone land (give it a speed along the negative z-axis)
  elif (state == 5):
    if ((pos_z > -0.05) and (pos_z < 0.05)):
      print("Initializing everything. Please wait...")
      state = 0
    else:
      state = 5

  return state

# This function reads an image from a topic, transforms it into a numpy array, shows the image on screen, runs the FSM
# functions and finally publishes the twist (velocity) on another topic.
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

  vel_pub.publish(twist)

# This function reads out the position from a topic and transforms the orientation (in quaternions) to a ypr representation
def callback_position(data):
  global pos_x, pos_y, pos_z, yaw, pitch, roll

  pos_x = data.pose.pose.position.x
  pos_y = data.pose.pose.position.y
  pos_z = data.pose.pose.position.z

  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, \
    data.pose.pose.orientation.z, data.pose.pose.orientation.w))

# This function reads out a boolean from a topic and stores it globally in target_lost
def callback_eval(data):
  global target_lost
  target_lost = data.data

if __name__=="__main__":
  rospy.init_node('simulate_random_behaviour', anonymous=True)
  try:
    rospy.Subscriber('/agent/kinect/kinect/rgb/image_raw', Image, callback_image)

    vel_pub = rospy.Publisher('/agent/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/agent/ground_truth/state', Odometry, callback_position)

    rospy.Subscriber('/eval', Bool, callback_eval)

    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
