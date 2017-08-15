cmake_minimum_required(VERSION 2.8.3)
project(mrvk_gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_msgs
)

find_package(Qt4 COMPONENTS QtCore QtGUi REQUIRED)

set(${PROJECT_NAME}_cpp_HDRS
        include/mrvk_gui/MrvkGuiPlugin.h
        )

set(${PROJECT_NAME}_cpp_SRCS
        src/MrvkGuiPlugin.cpp
        )
set(${PROJECT_NAME}_cpp_UIS
        uis/MainWidget.ui
        )

qt4_wrap_cpp(${PROJECT_NAME}_cpp_MOCS ${${PROJECT_NAME}_cpp_HDRS})
qt4_wrap_ui(${PROJECT_NAME}_cpp_UIS_H ${${PROJECT_NAME}_cpp_UIS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  include
)
include(${QT_USE_FILE})

add_library(${PROJECT_NAME}
        ${${PROJECT_NAME}_cpp_HDRS}
        ${${PROJECT_NAME}_cpp_SRCS}
        ${${PROJECT_NAME}_cpp_MOCS}
        ${${PROJECT_NAME}_cpp_UIS_H}
        )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${QT_QTCORE_LIBRARY}
        ${QT_QTGUI_LIBRARY}
        )
