cmake_minimum_required(VERSION 2.8.3)
project(mrvk_sidewalk)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
		cv_bridge
		pcl_ros
  		roscpp
  		rospy
		image_transport
		std_msgs
		tf
		tf_conversions
)
find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)
find_package(OpenCV REQUIRED)

catkin_package(
		INCLUDE_DIRS include
#  LIBRARIES mrvk_sidewalk
#  CATKIN_DEPENDS cv_bridge pcl_ros roscpp rospy
#  DEPENDS system_lib
)


include_directories(
		include/mrvk_sidewalk
   		${catkin_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
add_definitions(-std=c++11)
#add_definitions(${PCL_DEFINITIONS})


set(LOCAL_INCLUDES 	src/recognize_sidewalk/recognize_sidewalk.cpp
					src/pavement_to_marker/pavement_to_marker.cpp
		           	src/pavement_to_cloud/pavement_to_cloud.cpp
					src/misc_tools/misc_tools.cpp
					src/recognize_sidewalk/picture_segmentation.cpp
					src/RecognizeSidewalkParams.cpp
					src/RecognizeSidewalkParams.h
					src/cloud_processing.cpp
					src/recognize_sidewalk/SidewalkEdge.cpp
					src/recognize_sidewalk/SidewalkEdge.h
					)

add_executable(recognize_sidewalk_video src/recognize_sidewalk_video.cpp ${LOCAL_INCLUDES})
target_link_libraries(recognize_sidewalk_video ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(recognize_sidewalk recognize_sidewalk_generate_messages_cpp)

add_executable(recognize_sidewalk_kinect src/recognize_sidewalk_kinect.cpp ${LOCAL_INCLUDES})
target_link_libraries(recognize_sidewalk_kinect ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(recognize_sidewalk_kinect recognize_sidewalk_kinect_generate_messages_cpp)

add_executable(camera_calibration src/camera_calibration.cpp ${LOCAL_INCLUDES} ${OpenCV_LIBS})
target_link_libraries(camera_calibration ${catkin_LIBRARIES})