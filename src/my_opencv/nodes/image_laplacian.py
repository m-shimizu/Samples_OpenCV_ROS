#!/usr/bin/python3
import rospy
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

programName = 'image_X'

def cb_nop(data):
    pass

def showImage(img): # PLEASE USE THIS FOR DEBUG ONLY
    cv2.imshow(programName, img)
    cv2.waitKey()

def process_image(msg):
    try:
        # Subscribing the original image from the topic (NECESSARY)
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        # Convert 64F into 8U
        abs_64f = np.absolute(laplacian)
        img_8u  = np.uint8(abs_64f)
        # Convert 8U into BGR8
        #rsltimg = cv2.merge([img_8u, img_8u, img_8u])
        rsltimg = cv2.cvtColor(img_8u, cv2.COLOR_GRAY2BGR)
        # Publishing the processed image to the topic (NECESSARY)
        imgMsg = bridge.cv2_to_imgmsg(rsltimg, "bgr8")
        pub = rospy.Publisher('image_out', Image, queue_size=10)
        pub.publish(imgMsg)
    except Exception as err:
        print(err)

def start_node():
    # Preparing a window for putting sliders and/or viewing an image
    cv2.namedWindow(programName, cv2.WINDOW_FREERATIO)
    # Creating sliders (IF YOU NEED SLIDERS)
#    cv2.createTrackbar("Xresize(X10)", programName, int(xrsz * 10), 20, cb_nop)
#    cv2.createTrackbar("Yresize(X10)", programName, int(yrsz * 10), 20, cb_nop)
    # Initializing this program as a ROS NODE
    rospy.init_node(programName)
    rospy.loginfo(programName + ' node started')
    # Preparing to subscribe ROS topics
    rospy.Subscriber("image_in", Image, process_image)
    # Making an event loop
#    rospy.spin() # spin() is not including the cv2 event loop
    while not rospy.is_shutdown(): # This loop is for cv2.getTrackbarPos
        if cv2.waitKey(1) == 27: # The ESC KEY can stop this loop.
            break
    # Desttory the window, if it was created at the start of this program
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    try:
        start_node()
#        argc = len(rospy.myargv(argv=sys.argv))
#        if argc == 2:
#            xrsz = float(rospy.myargv(argv=sys.argv)[1])
#            yrsz = xrsz
#        elif argc == 3:
#            xrsz = float(rospy.myargv(argv=sys.argv)[1])
#            yrsz = float(rospy.myargv(argv=sys.argv)[2])
#        else:
#            xrsz = 1.0
#            yrsz = 1.0
#        start_node(xrsz, yrsz)
#        start_node( rospy.myargv(argv=sys.argv)[1])
    except rospy.ROSInterruptException:
        pass

