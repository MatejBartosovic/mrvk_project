cmake_minimum_required(VERSION 2.8.3)
project(vfh_local_planner)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  nav_core
  roscpp
  rospy
  std_msgs
  pluginlib
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vfh_local_planner
  CATKIN_DEPENDS base_local_planner costmap_2d nav_core roscpp rospy std_msgs pluginlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(vfh_local_planner src/vfh_planner.cpp src/vfh_planner_ros.cpp)
target_link_libraries(vfh_local_planner ${catkin_LIBRARIES})

install(TARGETS vfh_local_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES vfh_blp_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
