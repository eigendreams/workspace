cmake_minimum_required(VERSION 2.8.3)
project(volti)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf joy message_generation dynamic_reconfigure)

#catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/PIDvel.cfg
)

add_message_files(
	FILES
	float32_12.msg
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