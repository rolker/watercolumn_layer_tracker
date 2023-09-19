cmake_minimum_required(VERSION 3.5.1)
project(watercolumn_layer_tracker)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    marine_acoustic_msgs
    rosbag
    roscpp
    rqt_gui
    rqt_gui_cpp
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
)

set(CMAKE_AUTOMOC ON)
if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 COMPONENTS Core Widgets)

if (Qt5Widgets_FOUND)
    if (Qt5Widgets_VERSION VERSION_LESS 5.6.0)
        message(FATAL_ERROR "Minimum supported Qt5 version is 5.6!")
    endif()
else()
    message(SEND_ERROR "The Qt5Widgets library could not be found!")
endif(Qt5Widgets_FOUND)

set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/ping.cpp
    src/slice.cpp
)

set(HEADERS
    src/mainwindow.h
)

add_executable(watercolumn_layer_tracker ${HEADERS} ${SOURCES})

add_dependencies(watercolumn_layer_tracker ${catkin_EXPORTED_TARGETS})

qt5_use_modules(watercolumn_layer_tracker Widgets)

target_link_libraries(watercolumn_layer_tracker ${QT_LIBRARIES} ${catkin_LIBRARIES})

INSTALL(TARGETS watercolumn_layer_tracker RUNTIME DESTINATION bin)
