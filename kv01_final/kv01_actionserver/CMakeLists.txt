cmake_minimum_required(VERSION 2.8.3)
project(kv01_actionserver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  kv01_driver
  roscpp
  sensor_msgs
  trajectory_msgs
)


add_action_files(
   DIRECTORY action
   FILES
   move.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES kv01_interface kv01_actionserver
  CATKIN_DEPENDS actionlib_msgs kv01_driver roscpp sensor_msgs trajectory_msgs
  DEPENDS
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${kv01_driver_INCLUDE_DIRS}
)
#add_executable(kv01_actionserver_node src/kv01_actionserver_node.cpp)
#target_link_libraries(kv01_actionserver_node ${catkin_LIBRARIES} ${kv01_actionserver_LIBRARIES} ${kv01_driver_LIBRARIES})

#add_executable(kv01_actionserver_node_klb4fail src/kv01_actionserver_node_klb4fail.cpp)
#target_link_libraries(kv01_actionserver_node_klb4fail ${catkin_LIBRARIES} ${kv01_actionserver_LIBRARIES} ${kv01_driver_LIBRARIES})

add_executable(kv01_actionserver_point src/kv01_actionserver_point.cpp)
target_link_libraries(kv01_actionserver_point ${catkin_LIBRARIES} ${kv01_actionserver_LIBRARIES} ${kv01_driver_LIBRARIES})
add_dependencies(kv01_actionserver_point ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
