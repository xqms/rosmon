
cmake_minimum_required(VERSION 2.8)
project(rosmon)

find_package(catkin)

catkin_package()

catkin_install_python(
	PROGRAMS src/rosmon
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
