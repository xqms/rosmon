
cmake_minimum_required(VERSION 2.8)
project(rosmon)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cmake_modules
	roslib
)

catkin_package()

find_package(TinyXML REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(rosmon
	src/main.cpp
	src/node.cpp
	src/launch_config.cpp
)
target_link_libraries(rosmon
	${catkin_LIBRARIES}
	${TinyXML_LIBRARIES}
	yaml-cpp
	util
)
