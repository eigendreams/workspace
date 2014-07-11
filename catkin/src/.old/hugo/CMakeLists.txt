cmake_minimum_required(VERSION 2.8.3)
project(hugo)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs joy)
catkin_package(
	INCLUDE_DIRS scripts launch
)
include_directories(include ${catkin_INCLUDE_DIRS})