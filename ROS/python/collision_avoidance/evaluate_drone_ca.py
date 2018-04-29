#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# This function reads out the position from a topic and calculates the distance to the walls.
# It also reads out the speed from a topic and decides whether a collision occured or not and publishes the result on another topic
def callback_position(data):

  x = abs(data.pose.pose.position.x)
  y = abs(data.pose.pose.position.y + 7.5) # Express coordinates relative to the center of the room (0.0, -7.5, 0.0)

  outer_wall_distance = min((10 - y),(10 - x))
  inner_wall_distance = 5 - outer_wall_distance

  if (min(inner_wall_distance, outer_wall_distance) < 0.5):
    collision = True
  else:
    collision = False

  coll_pub.publish(collision)

if __name__=="__main__":
  rospy.init_node('evaluate_drone_ca', anonymous=True)
  try:
    rospy.Subscriber('/ground_truth/state', Odometry, callback_position)

    coll_pub = rospy.Publisher('/collision', Bool, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin()
  except rospy.ROSInterruptException:
        pass
