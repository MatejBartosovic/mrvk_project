cmake_minimum_required(VERSION 2.8.3)
project(mrvk_gui)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rqt_gui
    rqt_gui_cpp
    std_msgs
    osm_planner
    diagnostic_msgs
    move_base_msgs
    cv_bridge
)

set(${PROJECT_NAME}_cpp_HDRS
        include/mrvk_gui/MrvkGuiPlugin.h
        )

set(${PROJECT_NAME}_cpp_SRCS
        src/MrvkGuiPlugin.cpp
        )

set(${PROJECT_NAME}_cpp_UIS
        uis/MainWidget.ui
        uis/DiagnosticsWidget.ui
	    uis/ControlWidget.ui
        )

set(${PROJECT_NAME}_diagnostics_cpp_HDRS
        include/mrvk_gui/DiagnosticsWidget.h
        include/mrvk_gui/DiagnosticItem.h
        include/mrvk_gui/DiagnosticModel.h
        )

set(${PROJECT_NAME}_diagnostics_cpp_SRCS
        src/DiagnosticsWidget.cpp
        src/DiagnosticItem.cpp
        src/DiagnosticModel.cpp
        )

set(${PROJECT_NAME}_diagnostics_cpp_UIS
        uis/DiagnosticsWidget.ui
        )

#find_package(Qt4 COMPONENTS QtCore QtGUi REQUIRED)


if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    find_package(Qt5Widgets REQUIRED)
    qt5_wrap_cpp(${PROJECT_NAME}_cpp_MOCS ${${PROJECT_NAME}_cpp_HDRS})
    qt5_wrap_ui(${PROJECT_NAME}_cpp_UIS_H ${${PROJECT_NAME}_cpp_UIS})
    qt5_wrap_cpp(${PROJECT_NAME}_diagnostics_cpp_MOCS ${${PROJECT_NAME}_diagnostics_cpp_HDRS})
    qt5_wrap_ui(${PROJECT_NAME}_diagnostics_cpp_UIS_H ${${PROJECT_NAME}_diagnostics_cpp_UIS})
    set(qt_LIBRARIES Qt5::Widgets)
else()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
    qt4_wrap_cpp(${PROJECT_NAME}_cpp_MOCS ${${PROJECT_NAME}_cpp_HDRS})
    qt4_wrap_ui(${PROJECT_NAME}_cpp_UIS_H ${${PROJECT_NAME}_cpp_UIS})
    qt4_wrap_cpp(${PROJECT_NAME}_diagnostics_cpp_MOCS ${${PROJECT_NAME}_diagnostics_cpp_HDRS})
    qt4_wrap_ui(${PROJECT_NAME}_diagnostics_cpp_UIS_H ${${PROJECT_NAME}_diagnostics_cpp_UIS})
    set(qt_LIBRARIES ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

add_library(${PROJECT_NAME}_diagnostics
        ${${PROJECT_NAME}_diagnostics_cpp_HDRS}
        ${${PROJECT_NAME}_diagnostics_cpp_SRCS}
        ${${PROJECT_NAME}_diagnostics_cpp_MOCS}
        ${${PROJECT_NAME}_diagnostics_cpp_UIS_H}
        )

add_dependencies(${PROJECT_NAME}_diagnostics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_diagnostics
        ${catkin_LIBRARIES}
        ${qt_LIBRARIES}
        )

add_library(${PROJECT_NAME}
        ${${PROJECT_NAME}_cpp_HDRS}
        ${${PROJECT_NAME}_cpp_SRCS}
        ${${PROJECT_NAME}_cpp_MOCS}
        ${${PROJECT_NAME}_cpp_UIS_H}
        )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_diagnostics)

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_diagnostics
        ${catkin_LIBRARIES}
        ${qt_LIBRARIES}
        )
