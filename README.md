#OpenCV Samples in ROS  

## How to use this repository  

    $ cd  
    $ git clone https://github.com/m-shimizu/Samples_OpenCV_ROS  
    $ cd Samples_OpenCV_ROS  
    $ catkin_make  
    $ source devel/setup.bash  
    $ roslaunch my_opencv  optical_flow.launch  

## A sample for creating a new package working with OpenCV  
* catkin_create_pkg options:  

    $  catkin_create_pkg my_opencv cv_bridge image_transport roscpp sensor_msgs std_msgs  

* CMakeLists.txt options:  

    find_package(OpenCV REQUIRED)  
    target_link_libraries(BINARY_NAME ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )  
