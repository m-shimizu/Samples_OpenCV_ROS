#OpenCV Samples in ROS(Noetic) with Python3  

## How to prepare  

    $ cd  
    $ git clone https://github.com/m-shimizu/Samples_OpenCV_ROS  
    $ sudo apt install ros-noetic-usb-cam ros-noetic-libuvc-camera libzbar-dev python3-pip  
    $ pip install pyzbar pillow  
    $ cd Samples_OpenCV_ROS  
    $ catkin_make  
    $ source devel/setup.bash  
    $ roslaunch my_opencv  optical_flow.launch  

## Prepare an image file of your face for detecting face demo  
Create an image file including your face as "SampleFace.png" under ~/Samples_OpenCV_ROS/src/my_opencv/images.  

## Program descriptions  

* Nodes  

|Name|Description|Language|
|:---|:----------|:------:|
|optical_flow|Tracking moving objects|C++|
|image_binary.py|Binarization|Python|
|image_binaryS.py|Binarization with a slider|Python|
|image_Faces.py|Detecting faces|Python|
|image_pub.py|Publishing an image topic from an image file|Python|
|image_QR.py|Detecting QR and Bar codes|Python|
|image_resizeS.py|Resizing an image with sliders|Python|
|image_X.py|A template file|Python|

* Launch files(Samples for using above nodes)  

|Name|Description|
|:---|:-----------|
|optical_flow.launch|Start uvc_camera and optical_flow|
|start-image_pub-and-image_Faces.launch|Start image_pub, image_Faces, rqt_image_view|
|start-image_pub-and-image_QR.launch|Start image_pub, image_QR, rqt_image_view|
|start-image_pub.launch|Start image_pub|
|start-usb_cam-and-image_binaryS.launch|Start usb_cam, image_binaryS, rqt_image_view|

## A sample for creating a new package working with OpenCV  
* catkin_create_pkg options:  

    $  catkin_create_pkg my_opencv cv_bridge image_transport rospy roscpp sensor_msgs std_msgs  

* CMakeLists.txt options:  

    find_package(OpenCV REQUIRED)  
    target_link_libraries(BINARY_NAME ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )  

### SEE ALSO  
* [opencv/data/haarcascades](https://github.com/opencv/opencv/tree/master/data/haarcascades)  
* [ROBOTIS Turtlebot3 e-Manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)  
* [Robot Ignite Academy: Linux for Robotics](https://www.robotigniteacademy.com/ja/course/linux-for-robotics_40_0/)  
* [Robot Ignite Academy: Python 3 for Robotics](https://www.robotigniteacademy.com/ja/course/python-3-for-robotics_38_0/)  

EDITED: 26.Jul.2022  
