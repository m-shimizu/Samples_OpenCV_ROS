#!/usr/bin/python3
import rospy
import sys
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

programName = 'image_Faces'
haarcascadeName = ''

def cb_nop(data):
    pass

def showImage(img): # PLEASE USE THIS FOR DEBUG ONLY
    cv2.imshow(programName, img)
    cv2.waitKey(1)

def process_image(msg):
    try:
        # Subscribing the original image from the topic (NECESSARY)
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
#        showImage(orig)

        # Image Processing (NECESSARY) : Detecting Faces codes
        # Make the orig image into the gray scale
        gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        # Load the trained cascade data
        face_cascade = cv2.CascadeClassifier(haarcascadeName)
        # Detect faces and marking them
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        i = 0
        for (x, y, w, h) in faces:
            i += 1
            cv2.rectangle(orig, (x, y), (x + w, y + h), (255,0,255), 5)
            cv2.putText(orig, "Face"+str(i), (x, y - 5), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
        
        # Publishing the processed image to the topic (NECESSARY)
        imgMsg = bridge.cv2_to_imgmsg(orig, "bgr8")
        pub = rospy.Publisher('image_out', Image, queue_size=10)
        pub.publish(imgMsg)
    except Exception as err:
        print(err)

def start_node(fileName):
    global haarcascadeName
    haarcascadeName = fileName
    # Preparing a window for putting sliders and/or viewing an image
    cv2.namedWindow(programName, cv2.WINDOW_FREERATIO)
    # Creating sliders (IF YOU NEED A SLIDER)
#    cv2.createTrackbar("BinTh", programName, 128, 255, cb_nop)
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
        argc = len(rospy.myargv(argv=sys.argv))
        if argc == 2:
            fileName = rospy.myargv(argv=sys.argv)[1]
        else:
            fileName = '/home/USERNAME/Samples_OpenCV_ROS/src/my_opencv/nodes/haarcascade_frontalface_default.xml'
        start_node(fileName)
    except rospy.ROSInterruptException:
        pass

