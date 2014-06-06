cmake_minimum_required(VERSION 2.8.3)
project(rrhh)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs joy message_generation)

#catkin_python_setup()

add_message_files(
	FILES
	RRHH.msg
)
generate_messages(
	DEPENDENCIES
	std_msgs
)
catkin_package(
	INCLUDE_DIRS include msg scripts
	CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(brazo_console src/brazo_console.cpp)
#target_link_libraries(brazo_console ${catkin_LIBRARIES})

install(PROGRAMS
	scripts/joy.py
	scripts/tf.py
	scripts/rand.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
