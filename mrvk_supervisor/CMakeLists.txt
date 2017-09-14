cmake_minimum_required(VERSION 2.8.3)
project(mrvk_supervisor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mrvk_supervisor
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

add_executable(${PROJECT_NAME} src/mrvk_supervisor)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})