
add_library(rosmon_fmt
	src/format.cc
	src/posix.cc
)

target_include_directories(rosmon_fmt PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS rosmon_fmt
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
