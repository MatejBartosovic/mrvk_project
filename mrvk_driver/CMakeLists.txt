cmake_minimum_required(VERSION 2.8.3)
project(mrvk_driver)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  std_msgs
  urdf
  joint_limits_interface
  serial
  message_generation
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
   FILES
   Battery.msg
   Power_managment.msg
   Mb_status.msg
   Mcb_status.msg
   Error_log.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
        cfg/RobotDynParam.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_communication_interface
  CATKIN_DEPENDS controller_manager hardware_interface roscpp std_msgs urdf message_runtime
  DEPENDS
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_communication_interface src/communication_interface.cpp src/command.cpp src/conversions.cpp src/callbacks.cpp)
add_dependencies(${PROJECT_NAME}_communication_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mrvk_driver src/mrvk_driver.cpp src/hwInterface.cpp)
add_dependencies(mrvk_driver ${PROJECT_NAME}_communication_interface)

add_executable(robot_control src/robot_control.cpp)
add_dependencies(robot_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_communication_interface ${catkin_LIBRARIES})
target_link_libraries(mrvk_driver ${catkin_LIBRARIES} ${PROJECT_NAME}_communication_interface)
target_link_libraries(robot_control ${catkin_LIBRARIES})