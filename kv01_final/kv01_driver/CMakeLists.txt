cmake_minimum_required(VERSION 2.8.3)
project(kv01_driver)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  serial
)
## Generate services in the 'srv' folder
add_message_files(
  FILES
  joint_speed.msg
)

add_service_files(
  FILES
  serial_data.srv
  driver.srv
)

generate_messages(DEPENDENCIES std_msgs)
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kv01_driver
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs std_srvs
  DEPENDS
)

add_library(kv01_driver src/common_functions.cpp
)


include_directories(
  include                
  ${catkin_INCLUDE_DIRS}
  ${my_package_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)


add_library(common_functions src/common_functions.cpp)
target_link_libraries(common_functions ${catkin_LIBRARIES} ${kv01_driver_LIBRARIES})
add_dependencies(common_functions kv01_driver_generate_messages_cpp)

#add_executable(communication_servis src/communication_servis.cpp)
#target_link_libraries(communication_servis ${catkin_LIBRARIES}${kv01_driver_LIBRARIES})
#add_dependencies(kv01_driver communication_servis)

#add_executable(kv01_state_publisher src/kv01_state_publisher.cpp)
#target_link_libraries(kv01_state_publisher ${catkin_LIBRARIES}${kv01_driver_LIBRARIES} common_functions${kv01_driver_LIBRARIES})
#add_dependencies(kv01_driver kv01_state_publisher)

add_executable(kv01_driver_servis src/kv01_driver_servis.cpp)
target_link_libraries(kv01_driver_servis ${catkin_LIBRARIES}${kv01_driver_LIBRARIES} common_functions${kv01_driver_LIBRARIES})
add_dependencies(kv01_driver_servis kv01_driver_generate_messages_cpp)