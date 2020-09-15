#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# known pump geometry
#  - units are pixels (of half-size image)
PUMP_DIAMETER = 360
PISTON_DIAMETER = 90
PISTON_COUNT = 7

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def process_image(msg):
    try:
        # pass
        # convert sensor_msgs/Image to OpenCV Image
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        drawImg = orig
        # resize image (half-size) for easier processing
        resized = cv2.resize(orig, None, fx=2.0, fy=2.0)
        drawImg = resized
        # convert to single-channel image
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        # threshold grayscale to binary (black & white) image
        threshVal = 75
        ret,thresh = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY)
        drawImg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        # show results
        showImage(drawImg)
    except Exception as err:
        print err

def start_node():
    rospy.init_node('detect_pump')
    rospy.loginfo('detect_pump node started')
    rospy.Subscriber("image", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
