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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

generate_messages(DEPENDENCIES std_msgs)
 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller_manager hardware_interface roscpp std_msgs urdf message_runtime
  DEPENDS system_lib
  LIBRARIES ${PROJECT_NAME}_communication_interface
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_communication_interface src/communication_interface.cpp src/common_functions.cpp)
target_link_libraries(${PROJECT_NAME}_communication_interface ${catkin_LIBRARIES} ${${PROJECT_NAME}_LIBRARIES})

add_executable(kv01_driver src/kv01_driver.cpp src/hwInterface.cpp)
target_link_libraries(kv01_driver ${catkin_LIBRARIES} ${PROJECT_NAME}_communication_interface)
add_dependencies(kv01_driver ${PROJECT_NAME}_communication_interface)
add_dependencies(kv01_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

