cmake_minimum_required(VERSION 2.8.3)
project(finder)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf joy message_generation)

#catkin_python_setup()

add_message_files(
	FILES
	uint8_64.msg
	int16_64.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	INCLUDE_DIRS scripts
	CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(brazo_console src/brazo_console.cpp)
#target_link_libraries(brazo_console ${catkin_LIBRARIES})

#install(PROGRAMS
#	scripts/battery_monitor.py
#	scripts/odometry.py
#	scripts/joy.py
#	scripts/virtual_joy.py
#	scripts/twist_to_motors.py	
#	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)