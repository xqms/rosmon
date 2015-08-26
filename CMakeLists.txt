
cmake_minimum_required(VERSION 2.8)
project(rosmon)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cmake_modules
	roslib
	message_generation
	rospack
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror")

add_executable(rosmon
	src/main.cpp
	src/node.cpp
	src/launch_config.cpp
	src/ui.cpp
	src/husl/husl.c
	src/ros_interface.cpp
	src/package_registry.cpp
	src/fd_watcher.cpp
	src/logger.cpp
)
target_link_libraries(rosmon
	${catkin_LIBRARIES}
	${TinyXML_LIBRARIES}
	yaml-cpp
	util
)
add_dependencies(rosmon
	${PROJECT_NAME}_generate_messages_cpp
)

# GUI
find_package(Qt4 COMPONENTS QtGui QtCore)
find_package(rqt_gui_cpp)

if(QT_FOUND AND rqt_gui_cpp_FOUND)
	include_directories(${rqt_gui_cpp_INCLUDE_DIRS})
	include(${QT_USE_FILE})
	include_directories(${CMAKE_CURRENT_BINARY_DIR})

	qt4_wrap_cpp(MOC_SRCS
		src/gui/mon_gui.h
		src/gui/mon_model.h
	)

	qt4_wrap_ui(UI_HDRS
		src/gui/mon_gui.ui
	)

	add_library(rosmon_gui
		${UI_HDRS}
		${MOC_SRCS}
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

# Version 1.1 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(50-rosmon SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(TARGETS rosmon
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)