cmake_minimum_required(VERSION 2.8.3)
project(rosplan_tiago_params)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	rospy_message_converter
	rosplan_tiago_core_msgs
	geometry_msgs
	people_msgs
	tf2
	roslib
)

find_package(Boost REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(

)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
	nodes/location_name_server
	nodes/people_server
	nodes/location_marker_publisher
	nodes/pose_saver
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)