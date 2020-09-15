#!/usr/bin/env python
import rospy
import sys
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def showImage(img): # PLEASE USE THIS FOR DEBUG ONLY
    cv2.imshow('image', img)
    cv2.waitKey(1)

def process_image(msg):
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        #BELOWS ARE JUST SAMPLES!!
        #YOU CAN UNDERSTAND cvimg IS A VARIABLE FOR AN IMAGE PROCESSING RESULT.
        #  resize image (half-size) for easier processing
        #    cvimg = cv2.resize(orig, None, fx=2.0, fy=2.0)
        #IF THE IMAGE FORMAT IS NOT BGR, 
        #IT HAVE TO BE CONVERTED INTO BGR FORMAT
        #BEFORE PUBLISHING.
        #  convert to single-channel image
        #    cvimggray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        #    cvimg = cv2.cvtColor(cvimggray, cv2.COLOR_GRAY2BGR)
        imgMsg = bridge.cv2_to_imgmsg(cvimg, "bgr8")
        pub = rospy.Publisher('image_out', Image, queue_size=10)
        pub.publish(imgMsg)
    except Exception as err:
        print err

def start_node():
    rospy.init_node('image_X')
    rospy.loginfo('image_X node started')
    rospy.Subscriber("image_in", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
#        start_node( rospy.myargv(argv=sys.argv)[1])
    except rospy.ROSInterruptException:
        pass
