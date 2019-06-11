cmake_minimum_required(VERSION 2.8.3)
project(mrvk_gui_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        message_generation
        osm_planner
        move_base
        actionlib
        actionlib_msgs
        )

add_message_files(
        FILES
        GeoPoint.msg
)

add_service_files(
        FILES
        AddGeoWaypoint.srv
        AddGeoWaypoints.srv
        EditWaypoint.srv
        EraseWaypointsQueue.srv
        GetWaypointsQueue.srv
        SwapWaypoints.srv
)

add_action_files(
        FILES
        PerformWaypoints.action
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        actionlib_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mrvk_gui_interface
        CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(mrvk_gui_interface
        src/GuiInterface.cpp
        src/GuiInterfaceNode.cpp
        )

add_dependencies(mrvk_gui_interface ${mrvk_gui_interface_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mrvk_gui_interface
        ${catkin_LIBRARIES}
        )
