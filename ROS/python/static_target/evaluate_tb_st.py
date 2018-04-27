#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# Instantiate CvBridge
bridge = CvBridge()

# This function reads an image from a topic, transforms it into a numpy array, scales it to a smaller image andfinally 
# publishes target_lost on another topic.
def callback_image(data):
  try:
    # Convert your ROS Image message to OpenCV2
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    image = image[::10,::10,:]
  except CvBridgeError, e:
    print(e)

  target_lost = evaluate(image)

  eval_pub.publish(target_lost)

# This function checks the image for red pixels. If a red pixel is found target_lost is false, if not target_lost is true.
def evaluate(image):
  target_lost = False
  for row in image:
    for pixel in row:
      if ((pixel[2] > 100) and (pixel[1] < 10) and (pixel[0] < 10)):
        return target_lost
  target_lost = True
  return target_lost

if __name__=="__main__":
  rospy.init_node('evaluate_tb_st', anonymous=True)
  try:
    rospy.Subscriber('/camera/camera/rgb/image_raw', Image, callback_image)

    eval_pub = rospy.Publisher('/eval', Bool, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin()
  except rospy.ROSInterruptException:
        pass
