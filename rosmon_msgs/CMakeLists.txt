
cmake_minimum_required(VERSION 2.8)
project(rosmon_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
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

