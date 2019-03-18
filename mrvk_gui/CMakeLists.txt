cmake_minimum_required(VERSION 2.8.3)
project(mrvk_gui)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rqt_gui
        rqt_gui_cpp
        )

#main ui sources
set(${PROJECT_NAME}_cpp_HDRS
        ui/MainWindow.h
        ui/Widgets/Camera/Camera.h
        ui/Widgets/Gps/Gps.h
        ui/Widgets/Imu/Imu.h
        ui/Widgets/MoveBase/MoveBase.h
        ui/Widgets/MoveBase/MoveBaseControl.h
        ui/Widgets/MoveBase/MoveBaseStatus.h
        ui/Widgets/Odometry/Odometry.h
        ui/Widgets/TopicFrequency/TopicFrequency.h
        ui/Widgets/MrvkDriver/MainBoardStatus.h
        ui/Widgets/MrvkDriver/MrvkDriver.h
        )

set(${PROJECT_NAME}_cpp_SRCS
        )

set(${PROJECT_NAME}_cpp_UIS
        ui/MainWindow.ui
        ui/Widgets/Camera/Camera.ui
        ui/Widgets/Gps/Gps.ui
        ui/Widgets/Imu/Imu.ui
        ui/Widgets/MoveBase/MoveBaseControl.ui
        ui/Widgets/MoveBase/MoveBaseStatus.ui
        ui/Widgets/MoveBase/MoveBase.ui
        ui/Widgets/Odometry/Odometry.ui
        ui/Widgets/TopicFrequency/TopicFrequency.ui
        ui/Widgets/MrvkDriver/MainBoardStatus.ui
        ui/Widgets/MrvkDriver/MrvkDriver.ui
        )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    find_package(Qt5Widgets REQUIRED)
    # Main ui mocs
    qt5_wrap_cpp(${PROJECT_NAME}_cpp_MOCS ${${PROJECT_NAME}_cpp_HDRS})
    qt5_wrap_ui(${PROJECT_NAME}_cpp_UIS_H ${${PROJECT_NAME}_cpp_UIS})
    set(qt_LIBRARIES Qt5::Widgets)
else()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    # Main ui mocs
    qt4_wrap_cpp(${PROJECT_NAME}_cpp_MOCS ${${PROJECT_NAME}_cpp_HDRS})
    qt4_wrap_ui(${PROJECT_NAME}_cpp_UIS_H ${${PROJECT_NAME}_cpp_UIS})
    include(${QT_USE_FILE})
    set(qt_LIBRARIES ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs
        #  DEPENDS system_lib
)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        include
        ui
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
        src/main.cpp
        ui/MainWindow.cpp
        ui/Widgets/Camera/Camera.cpp
        ui/Widgets/Gps/Gps.cpp
        ui/Widgets/Imu/Imu.cpp
        ui/Widgets/MoveBase/MoveBase.cpp
        ui/Widgets/MoveBase/MoveBaseControl.cpp
        ui/Widgets/MoveBase/MoveBaseStatus.cpp
        ui/Widgets/Odometry/Odometry.cpp
        ui/Widgets/TopicFrequency/TopicFrequency.cpp
        ui/Widgets/MrvkDriver/MainBoardStatus.cpp
        ui/Widgets/MrvkDriver/MrvkDriver.cpp
        ${${PROJECT_NAME}_cpp_MOCS}
        ${${PROJECT_NAME}_cpp_UIS_H})


target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${qt_LIBRARIES}
        )