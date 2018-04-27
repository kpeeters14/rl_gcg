#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import numpy as np
import tf

state = 0

pos_x = 0
pos_y = 0

yaw = 0
pitch = 0
roll = 0

radius = 0

target_yaw = 0
circle_yaw = 0

target_lost = False
ready_for_action = False

# This function is a FSM that tells the target what to do in each state and returns a twist (velocity) message
def fsm_datapath():
  global circle_yaw
  twist = Twist()

  # RESET & INITIALIZE
  # Reset the target (make it stop at its current position)
  # Wait until everything is ready
  if (state == 0):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

  # MOVE
  # Make the target move in a circle around the agent
  if (state == 1):
    twist.linear.x = 0.5
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = twist.linear.x/3  # 1/3 of linear speed so it goes in a circle with radius 3

  # RESET RADIUS: FACE AGENT
  # Make the target face the same direction as the agent
  if (state == 2):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    if (abs(target_yaw - yaw) > 0.02):
      twist.angular.z = -0.25
    else:
      twist.angular.z = 0.0
    circle_yaw = target_yaw + 1.57
    if (circle_yaw > 3.14):
      circle_yaw = -(3.14 - (circle_yaw - 3.14))

  # RESET RADIUS: FIX RADIUS
  # Make the target move in the direction of the agent to reduce the distance between them (radius)
  if (state == 3):
    if (radius < 2.95):
      twist.linear.x = 0.25
    elif (radius > 3.05):
      twist.linear.x = -0.25
    else:
      twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

  # RESET RADIUS: FACE CIRCLE
  # Make the target face its original direction in order to move in the circle again
  if (state == 4):
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    if (abs(circle_yaw - yaw) > 0.02):
      twist.angular.z = 0.25
    else:
      twist.angular.z = 0.0

  return twist

# This function is a FSM that takes care of the state transitions for the target and returns the state.
def fsm_controller():
  global state

  # RESET & INITIALIZE
  # Reset the target (make it stop at its current position)
  # Wait until everything is ready
  if (state == 0):
    if (ready_for_action):
      state = 1
    else:
      state = 0

  # MOVE
  # Make the target move in a circle around the agent
  elif (state == 1):
    if (target_lost):
      if ((radius > 4) or (radius < 2)):
        state = 2
      else:
        state = 0
    else:
      state = 1

  # RESET RADIUS: FACE AGENT
  # Make the target face the same direction as the agent
  if (state == 2):
    if (abs(target_yaw - yaw) < 0.02):
      state = 3
    else:
      state = 2

  # RESET RADIUS: FIX RADIUS
  # Make the target move in the direction of the agent to reduce the distance between them (radius)
  if (state == 3):
    if (radius < 3.05):
      state = 4
    else:
      state = 3

  # RESET RADIUS: FACE CIRCLE
  # Make the target face its original direction in order to move in the circle again
  if (state == 4):
    if (abs(circle_yaw - yaw) < 0.02):
      state = 0
    else:
      state = 4

  return state

# This function reads out the position from a topic
def callback_position(data):
  global pos_x, pos_y

  pos_x = data.pose.pose.position.x
  pos_y = data.pose.pose.position.y

# This function reads out the target position from a topic and calculates the distance between the agent and target (radius)
def callback_target_position(data):
  global radius, target_yaw, yaw, pitch, roll

  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, \
    data.pose.pose.orientation.z, data.pose.pose.orientation.w)) 

  target_pos_x = 3 - (data.pose.pose.position.y - pos_x)  # Axes from target are not the same as those from agent 
  target_pos_y = data.pose.pose.position.x                # --> conversion to express target coordinates relative to agent axes

  radius = np.sqrt((target_pos_x**2) + (target_pos_y**2))

  target_yaw = np.arctan(max(abs(target_pos_y), 0.001)/max(abs(target_pos_x), 0.001))
  if ((target_pos_x < 0) and (target_pos_y > 0)):
    target_yaw = 3.14 - target_yaw
  elif ((target_pos_x < 0) and (target_pos_y < 0)):
    target_yaw = -3.14 + target_yaw
  elif ((target_pos_x > 0) and (target_pos_y < 0)):
    target_yaw = -target_yaw

  target_yaw = target_yaw - 1.57
  if (target_yaw < -3.14):
    target_yaw = 3.14 - (abs(target_yaw) - 3.14)

# This function reads out a boolean from a topic and stores it globally in target_lost
# Afterwards it calls the FSM functions to change the state and twist (velocity) and finally it publishes the twist
def callback_eval(data):
  global target_lost
  target_lost = data.data

  state = fsm_controller()
  twist = fsm_datapath()

  vel_pub.publish(twist)

# This function reads out a boolean from a topic and stores it globally in ready_for_action
def callback_ready(data):
  global ready_for_action
  ready_for_action = data.data

if __name__=="__main__":
  rospy.init_node('behaviour_target', anonymous=True)
  try:
    vel_pub = rospy.Publisher('/target/mobile_base/commands/velocity', Twist, queue_size=10)

    rospy.Subscriber('/agent/ground_truth/state', Odometry, callback_position)

    rospy.Subscriber('/target/odom', Odometry, callback_target_position)

    rospy.Subscriber('/eval', Bool, callback_eval)

    rospy.Subscriber('/ready_for_action', Bool, callback_ready)

    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin()
  except rospy.ROSInterruptException:
        pass
