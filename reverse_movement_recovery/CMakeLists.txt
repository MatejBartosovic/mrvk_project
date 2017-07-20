cmake_minimum_required(VERSION 2.8.3)
project(reverse_movement_recovery)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  costmap_2d
  nav_core
  pluginlib
  roscpp
  tf
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rotate_recovery
        CATKIN_DEPENDS
        roscpp
        pluginlib
        geometry_msgs
)

###########
## Build ##
###########

find_package(Eigen REQUIRED)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        include
)

add_library(reverse_movement_recovery src/reverse_movement_recovery.cpp)
target_link_libraries(reverse_movement_recovery ${catkin_LIBRARIES})

