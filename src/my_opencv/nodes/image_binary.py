#!/usr/bin/python3
import rospy
import sys
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

programName = 'image_binary'

def cb_nop(data):
    pass

def showImage(img): # PLEASE USE THIS FOR DEBUG ONLY
    cv2.imshow(programName, img)
#    cv2.waitKey(1)

def process_image(msg):
    try:
        # Subscribing the original image from the topic (NECESSARY)
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Image Processing (NECESSARY) : BINARIZE
        gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        binth = cv2.getTrackbarPos("BinTh", programName)
        _, cvbinimg = cv2.threshold(gray, binth, 255, cv2.THRESH_BINARY)
        cvimg = cv2.cvtColor(cvbinimg, cv2.COLOR_GRAY2BGR)

        # Publishing the processed image to the topic (NECESSARY)
        imgMsg = bridge.cv2_to_imgmsg(cvimg, "bgr8")
        pub = rospy.Publisher('image_out', Image, queue_size=10)
        pub.publish(imgMsg)
    except Exception as err:
        print(err)

def start_node():
    # Preparing a window for putting sliders and/or viewing an image
    cv2.namedWindow(programName, cv2.WINDOW_FREERATIO)
    # Creating sliders (IF YOU NEED A SLIDER)
    cv2.createTrackbar("BinTh", programName, 128, 255, cb_nop)
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
#        start_node( rospy.myargv(argv=sys.argv)[1])
    except rospy.ROSInterruptException:
        pass

