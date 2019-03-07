
cmake_minimum_required(VERSION 2.8)
project(rqt_rosmon)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pluginlib
	rosmon_msgs
	rqt_gui
	rqt_gui_cpp
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror")

if(${pluginlib_VERSION} VERSION_GREATER 1.11.1)
	message(STATUS "using new pluginlib headers with .hpp extensions")
	add_definitions(-DHAVE_PLUGINLIB_NEW_HEADERS=1)
endif()

# Decide on a Qt version
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
	set(USE_QT5 true)
else()
	set(USE_QT5 false)
endif()

# Find Qt
if(USE_QT5)
	find_package(Qt5Widgets REQUIRED)
	set(QT_FOUND ${Qt5Widgets_FOUND})
	set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})
else()
	find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
	include(${QT_USE_FILE})
endif()

if(NOT QT_FOUND)
	message(FATAL_ERROR "Could not find Qt!")
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(UI_FILES
	src/mon_gui.ui
)
set(H_FILES
	src/mon_gui.h
	src/node_model.h
	src/rosmon_model.h
)

if(USE_QT5)
	qt5_wrap_ui(UIC_FILES ${UI_FILES})
	qt5_wrap_cpp(MOC_FILES ${H_FILES})
else()
	qt4_wrap_ui(UIC_FILES ${UI_FILES})
	qt4_wrap_cpp(MOC_FILES ${H_FILES})
endif()

add_library(rqt_rosmon
	${UIC_FILES}
	${MOC_FILES}
	src/bar_delegate.cpp
	src/format_data_size.cpp
	src/mon_gui.cpp
	src/node_model.cpp
	src/rosmon_model.cpp
)
target_link_libraries(rqt_rosmon
	${catkin_LIBRARIES}
	${rqt_gui_cpp_LIBRARIES}
	${QT_LIBRARIES}
)

install(TARGETS rqt_rosmon
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES rqt_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/rqt_rosmon
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
