cmake_minimum_required(VERSION 2.8.3)
project(rosmon_diagnostics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Werror)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_DEPS roscpp rosmon_msgs diagnostic_msgs rosfmt)
find_package(catkin REQUIRED ${CATKIN_DEPS})

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
aux_source_directory(src NODE_SRC)
add_executable(rosmon_diagnostics_node ${NODE_SRC})
add_dependencies(rosmon_diagnostics_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosmon_diagnostics_node ${catkin_LIBRARIES})

#############
## Install ##
#############
# Mark executables and/or libraries for installation
 install(TARGETS rosmon_diagnostics_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
find_package(rostest REQUIRED)
set(TEST_SRC ${NODE_SRC})
list(REMOVE_ITEM TEST_SRC src/main.cpp)

add_rostest_gtest(rosmon_diagnostics-test test/rosmon_diagnostics.test test/rosmon_diagnostics.test.cc ${TEST_SRC})
target_link_libraries(rosmon_diagnostics-test ${catkin_LIBRARIES})

# A small node usefull for testing diagnostics :
add_executable(node_for_test test/node_for_test.cpp)
add_dependencies(node_for_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_for_test ${catkin_LIBRARIES})
