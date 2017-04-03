cmake_minimum_required(VERSION 2.8.3)
project(kv01_driver)


find_package(catkin REQUIRED COMPONENTS
        controller_manager
        hardware_interface
        roscpp
        std_msgs
        urdf
        joint_limits_interface
        transmission_interface
        serial
        message_generation
        dynamic_reconfigure
)
## Generate services in the 'srv' folder

generate_messages(DEPENDENCIES std_msgs)
 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller_manager hardware_interface roscpp std_msgs urdf message_runtime
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${my_package_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_communication_interface src/communication_interface.cpp)
target_link_libraries(${PROJECT_NAME}_communication_interface ${catkin_LIBRARIES})

add_executable(kv01_driver src/kv01_driver.cpp src/hwInterface.cpp)
target_link_libraries(kv01_driver ${catkin_LIBRARIES} ${PROJECT_NAME}_communication_interface)

add_dependencies(kv01_driver ${PROJECT_NAME}_communication_interface)

