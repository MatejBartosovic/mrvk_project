cmake_minimum_required(VERSION 2.8.3)
project(ximea_camera)



## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg image_transport cv_bridge)

## Declare ROS messages and services
add_message_files(FILES Num.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(/usr/local/lib)
add_executable(ximea_publisher src/ximea_camera.cpp src/xiApiPlusOcv.cpp)
target_link_libraries(ximea_publisher ${catkin_LIBRARIES} m3api opencv_core opencv_imgproc opencv_highgui opencv_flann opencv_imgcodecs opencv_features2d)
add_dependencies(ximea_publisher ximea_camera_generate_messages_cpp)

