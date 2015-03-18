cmake_minimum_required(VERSION 2.8.3)
project(arm_interface)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs joy message_generation)

#catkin_python_setup()

add_message_files(
	FILES
	Arm.msg
)
generate_messages(
	DEPENDENCIES
	std_msgs
)
catkin_package(
	INCLUDE_DIRS msg scripts
	CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
#add_executable(brazo_console src/brazo_console.cpp)
#target_link_libraries(brazo_console ${catkin_LIBRARIES})

install(PROGRAMS
	scripts/gui_simple.py
	scripts/gui_simple.glade
	scripts/shell.py
	scripts/joy.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)