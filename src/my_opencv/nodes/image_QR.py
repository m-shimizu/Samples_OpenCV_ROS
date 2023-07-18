#!/usr/bin/python3
import rospy
import sys
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

programName = 'image_QR'
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

    # Image Processing (NECESSARY) : Detecting QR codes
    for barcode in decode(orig):
      print(barcode.data)
#      showImage(orig)
      myData = barcode.data.decode('utf-8')
      print(myData)
      points = np.array([barcode.polygon], np.int32)
      points = points.reshape((-1, 1, 2))
      cv2.polylines(orig, [points], True, (255,0,255), 5)
      points_rectangle = barcode.rect
      cv2.putText(orig, myData, (points_rectangle[0], points_rectangle[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
    
    # Publishing the processed image to the topic (NECESSARY)
    imgMsg = bridge.cv2_to_imgmsg(orig, "bgr8")
    pub.publish(imgMsg)
  except Exception as err:
    print(err)

def start_node():
  global programName, pub
  # Preparing a window for putting sliders and/or viewing an image
  cv2.namedWindow(programName, cv2.WINDOW_FREERATIO)
  # Creating sliders (IF YOU NEED A SLIDER)
#  cv2.createTrackbar("BinTh", programName, 128, 255, cb_nop)
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
    start_node()
#    start_node( rospy.myargv(argv=sys.argv)[1])
  except rospy.ROSInterruptException:
    pass

