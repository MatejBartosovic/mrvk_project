cmake_minimum_required(VERSION 2.8.3)
project(kv01_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kv01_driver
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  joy
)
catkin_package(
  INCLUDE_DIRS
  LIBRARIES kv01_interface
  CATKIN_DEPENDS joy kv01_driver roscpp sensor_msgs std_msgs std_srvs
  DEPENDS
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${kv01_driver_INCLUDE_DIRS}
)

add_executable(kv01_menu src/kv01_menu.cpp)
target_link_libraries(kv01_menu ${catkin_LIBRARIES} ${kv01_interface_LIBRARIES} ${kv01_driver_LIBRARIES})
add_dependencies(kv01_menu kv01_driver_generate_messages_cpp)

add_executable(kv01_joystick src/kv01_joystick.cpp)
target_link_libraries(kv01_joystick ${catkin_LIBRARIES} ${kv01_interface_LIBRARIES} ${kv01_driver_LIBRARIES})
add_dependencies(kv01_joystick kv01_driver_generate_messages_cpp)

