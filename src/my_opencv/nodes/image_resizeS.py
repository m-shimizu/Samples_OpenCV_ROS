#!/usr/bin/python3
import rospy
import sys
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

programName = 'image_resize'
pub = 0

def cb_nop(data):
  pass

def showImage(img): # PLEASE USE THIS FOR DEBUG ONLY
  global programName
  cv2.imshow(programName, img)
  cv2.waitKey(1)

def process_image(msg):
  global programName, pub
  try:
    # Subscribing the original image from the topic (NECESSARY)
    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")

#    showImage(orig)

    # Image Processing (NECESSARY) : Detecting Faces codes
    fxi = cv2.getTrackbarPos("Xresize(X10)", programName)
    fyi = cv2.getTrackbarPos("Yresize(X10)", programName)
    if fxi == 0:
      fx_ = 0.1
    else:
      fx_ = float(fxi) / 10.0
    if fyi == 0:
      fy_ = 0.1
    else:
      fy_ = float(fyi) / 10.0
    resized = cv2.resize(orig, None, fx=fx_, fy=fy_)

    # Publishing the processed image to the topic (NECESSARY)
    imgMsg = bridge.cv2_to_imgmsg(resized, "bgr8")
    pub.publish(imgMsg)
  except Exception as err:
    print(err)

def start_node(xrsz, yrsz):
  global programName, pub
  # Preparing a window for putting sliders and/or viewing an image
  cv2.namedWindow(programName, cv2.WINDOW_FREERATIO)
  # Creating sliders (IF YOU NEED A SLIDER)
  cv2.createTrackbar("Xresize(X10)", programName, int(xrsz * 10), 20, cb_nop)
  cv2.createTrackbar("Yresize(X10)", programName, int(yrsz * 10), 20, cb_nop)
  # Initializing this program as a ROS NODE
  rospy.init_node(programName)
  rospy.loginfo(programName + ' node started')
  # Preparing to subscribe and publish ROS topics
  rospy.Subscriber("image_in", Image, process_image)
  pub = rospy.Publisher('image_out', Image, queue_size=10)
  # Making an event loop
#  rospy.spin() # spin() is not including the cv2 event loop
  while not rospy.is_shutdown(): # This loop is for cv2.getTrackbarPos
    if cv2.waitKey(1) == 27: # The ESC KEY can stop this loop.
      break
  # Desttory the window, if it was created at the start of this program
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  try:
    argc = len(rospy.myargv(argv=sys.argv))
    if argc == 2:
      xrsz = float(rospy.myargv(argv=sys.argv)[1])
      yrsz = xrsz
    elif argc == 3:
      xrsz = float(rospy.myargv(argv=sys.argv)[1])
      yrsz = float(rospy.myargv(argv=sys.argv)[2])
    else:
      xrsz = 1.0
      yrsz = 1.0
    start_node(xrsz, yrsz)
#    start_node( rospy.myargv(argv=sys.argv)[1])
  except rospy.ROSInterruptException:
    pass

