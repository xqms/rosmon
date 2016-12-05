
cmake_minimum_required(VERSION 2.8)
project(rosmon)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cmake_modules
	roslib
	message_generation
	rospack
	rqt_gui
	rqt_gui_cpp
)

add_message_files(FILES
	NodeState.msg
	State.msg
)

add_service_files(FILES
	StartStop.srv
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package()

find_package(TinyXML REQUIRED)

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror")

add_executable(rosmon
	src/main.cpp
	src/launch/node.cpp
	src/launch/launch_config.cpp
	src/monitor/node_monitor.cpp
	src/monitor/monitor.cpp
	src/ui.cpp
	src/husl/husl.c
	src/ros_interface.cpp
	src/package_registry.cpp
	src/fd_watcher.cpp
	src/logger.cpp
	src/terminal.cpp
)
target_link_libraries(rosmon
	${catkin_LIBRARIES}
	${TinyXML_LIBRARIES}
	${CURSES_LIBRARIES}
	yaml-cpp
	util
)
add_dependencies(rosmon
	${PROJECT_NAME}_generate_messages_cpp
)

# GUI

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

find_package(rqt_gui_cpp)

if(QT_FOUND AND rqt_gui_cpp_FOUND)

	include_directories(${rqt_gui_cpp_INCLUDE_DIRS})
	include_directories(${CMAKE_CURRENT_BINARY_DIR})

	set(UI_FILES
		src/gui/mon_gui.ui
	)
	set(H_FILES
		src/gui/mon_gui.h
		src/gui/mon_model.h
	)

	if(USE_QT5)
		qt5_wrap_ui(UIC_FILES ${UI_FILES})
		qt5_wrap_cpp(MOC_FILES ${H_FILES})
	else()
		qt4_wrap_ui(UIC_FILES ${UI_FILES})
		qt4_wrap_cpp(MOC_FILES ${H_FILES})
	endif()

	add_library(rosmon_gui
		${UIC_FILES}
		${MOC_FILES}
		src/gui/mon_gui.cpp
		src/gui/mon_model.cpp
	)
	target_link_libraries(rosmon_gui
		${catkin_LIBRARIES}
		${rqt_gui_cpp_LIBRARIES}
		${QT_LIBRARIES}
	)
	add_dependencies(rosmon_gui
		${PROJECT_NAME}_generate_messages_cpp
	)
endif()

# Utils
add_executable(abort
	src/util/abort.cpp
)

add_executable(abort_really_long_executable
	src/util/abort.cpp
)

# Register unit tests
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(test/basic.test)
endif()

# Version 1.4 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(50-rosmon
	SHELLS bash zsh
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

install(TARGETS rosmon
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
