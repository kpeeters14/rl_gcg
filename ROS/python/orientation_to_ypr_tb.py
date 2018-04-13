#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import tf

# This function reads out the position from a topic and transforms the orientation (in quaternions) to a ypr representation
def callback_position(data):
  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, \
    data.pose.pose.orientation.z, data.pose.pose.orientation.w))

  ypr = Vector3()
  ypr.x = yaw
  ypr.y = pitch
  ypr.z = roll

  ypr_pub.publish(ypr)

if __name__=="__main__":
  rospy.init_node('orientation_to_ypr_tb', anonymous=True)
  try:
    rospy.Subscriber('/odom', Odometry, callback_position)

    ypr_pub = rospy.Publisher('/odom/ypr', Vector3, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin()
  except rospy.ROSInterruptException:
        pass
