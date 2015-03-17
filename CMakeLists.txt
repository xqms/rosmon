
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

# Version 1.1 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(50-rosmon SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

