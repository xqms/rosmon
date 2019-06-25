
cmake_minimum_required(VERSION 2.8)
project(rosmon_core)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cmake_modules
	roslib
	rosfmt
	rosmon_msgs
	rospack
	diagnostic_msgs
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror")

find_package(TinyXML REQUIRED)

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIRS})

# We search for the same Python version that catkin has decided on.
# Source: https://github.com/ros/rospack/blob/70eac5dec07311f9cacccddb301a8bc9b4efb671/CMakeLists.txt#L6
set(Python_ADDITIONAL_VERSIONS "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" QUIET)
if(PYTHONLIBS_FOUND)
	add_definitions(-DHAVE_PYTHON=1)
	include_directories(${PYTHON_INCLUDE_DIRS})
else()
	message(WARNING "Please install libpython-dev (or equivalent) for $(eval ...) support")
endif()

find_package(Boost REQUIRED COMPONENTS python REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Specific feature tests
if(${roscpp_VERSION} VERSION_GREATER 1.12.8)
	message(STATUS "roscpp is new enough, using SteadyTimer for timekeeping")
	add_definitions(-DHAVE_STEADYTIMER=1)
endif()

# Are we building with test coverage instrumentation?
set(BUILD_FOR_COVERAGE OFF CACHE BOOL "Build with coverage instrumentation?")
if(BUILD_FOR_COVERAGE)
	if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
	else()
		message(WARNING "Coverage build is only supported with clang")
	endif()
endif()

add_library(rosmon_launch_config
	src/launch/node.cpp
	src/launch/launch_config.cpp
	src/launch/substitution.cpp
	src/launch/substitution_python.cpp
	src/launch/yaml_params.cpp
	src/launch/bytes_parser.cpp
	src/launch/string_utils.cpp
	src/package_registry.cpp
)
target_link_libraries(rosmon_launch_config
	${catkin_LIBRARIES}
	${TinyXML_LIBRARIES}
	${Boost_LIBRARIES}
	${Python_LIBRARIES}
	yaml-cpp
)

add_executable(_shim
	src/monitor/shim.cpp
)
target_link_libraries(_shim
	util
)

add_executable(rosmon
	src/main.cpp
	src/monitor/node_monitor.cpp
	src/monitor/monitor.cpp
	src/monitor/linux_process_info.cpp
	src/diagnostics_publisher.cpp
	src/ui.cpp
	src/husl/husl.c
	src/ros_interface.cpp
	src/fd_watcher.cpp
	src/logger.cpp
	src/terminal.cpp
)
target_link_libraries(rosmon
	${catkin_LIBRARIES}
	${TinyXML_LIBRARIES}
	${CURSES_LIBRARIES}
	${Boost_LIBRARIES}
	yaml-cpp
	util
	rosmon_launch_config
)

add_dependencies(rosmon ${catkin_EXPORTED_TARGETS})

if(PYTHONLIBS_FOUND)
	target_link_libraries(rosmon
		${PYTHON_LIBRARIES}
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
	# Integration tests
	find_package(rostest REQUIRED)
	add_rostest(test/basic.test DEPENDENCIES rosmon)

	# XML parsing test suite
	find_package(catch_ros)
	if(catch_ros_FOUND)
		include_directories(${catch_ros_INCLUDE_DIRS})

		catch_add_test(test_xml_loading
			test/xml/node_utils.cpp
			test/xml/test_arg.cpp
			test/xml/test_env.cpp
			test/xml/test_basic.cpp
			test/xml/test_if_unless.cpp
			test/xml/test_include.cpp
			test/xml/test_node.cpp
			test/xml/test_param.cpp
			test/xml/test_remap.cpp
			test/xml/test_rosparam.cpp
			test/xml/test_subst.cpp
			test/xml/test_memory.cpp
		)
		target_link_libraries(test_xml_loading
			rosmon_launch_config
			${catch_ros_LIBRARIES}
		)
	else()
		message(WARNING "Install catch_ros to enable XML unit tests")
	endif()
endif()

# Version 1.5 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(50-rosmon
	SHELLS bash zsh
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

install(TARGETS rosmon rosmon_launch_config
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
