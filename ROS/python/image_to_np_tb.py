#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()

# This function reads an image from a topic, transforms it into a numpy array, downscales it and finally publishes the 
# image as a numpy array on another topic.
def callback_image(data):
  try:
    # Convert your ROS Image message to OpenCV2
    image = bridge.imgmsg_to_cv2(data, "mono8")
    image = image[::10,::10]  # Downscale the image to a 36*64*1 image
  except CvBridgeError, e:
    print(e)

  image = image.flatten()
  image = image.astype(np.float32)

  np_pub.publish(image)

if __name__=="__main__":
  rospy.init_node('image_to_np_tb', anonymous=True)
  try:
    rospy.Subscriber('/agent/camera/camera/rgb/image_raw', Image, callback_image)

    np_pub = rospy.Publisher('/agent/camera/camera/np_image', numpy_msg(Floats), queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin()
  except rospy.ROSInterruptException:
        pass
