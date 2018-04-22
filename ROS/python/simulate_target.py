#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

state = 0
target_lost = False
ready_for_action = False

# This function is a FSM that tells the target what to do in each state and returns a twist (velocity) message
def fsm_datapath():
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
    twist.linear.x = 1.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = twist.linear.x/3  # 1/3 of linear speed so it goes in a circle with radius 3

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
      state = 0
    else:
      state = 1

  return state

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
  rospy.init_node('simulate_target', anonymous=True)
  try:
    vel_pub = rospy.Publisher('/target/mobile_base/commands/velocity', Twist, queue_size=10)

    rospy.Subscriber('/eval', Bool, callback_eval)

    rospy.Subscriber('/ready_for_action', Bool, callback_ready)

    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin()
  except rospy.ROSInterruptException:
        pass
