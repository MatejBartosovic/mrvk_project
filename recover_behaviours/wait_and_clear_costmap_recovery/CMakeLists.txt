cmake_minimum_required(VERSION 2.8.3)
project(wait_and_clear_costmap_recovery)

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
)

find_package(Eigen3 REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include
)

add_library(wait_and_clear_costmap_recovery src/wait_and_clear_costmap_recovery.cpp)
target_link_libraries(wait_and_clear_costmap_recovery ${catkin_LIBRARIES})
