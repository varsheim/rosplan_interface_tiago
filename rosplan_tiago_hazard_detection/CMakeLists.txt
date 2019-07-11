cmake_minimum_required(VERSION 2.8.3)
project(rosplan_tiago_hazard_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	diagnostic_msgs
	geometry_msgs
	roscpp
	rosplan_dispatch_msgs
	rosplan_knowledge_msgs
	rosplan_planning_system
	rosplan_tiago_params
	rosplan_tiago_common
	rospy
	std_msgs
	std_srvs
	tf
	genmsg
	actionlib
	actionlib_msgs
	smach
)


find_package(Boost REQUIRED COMPONENTS thread)

## Generate actions in the 'action' folder
add_action_files(FILES Go.action Check.action)

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	roscpp
	rospy
	std_msgs
	std_srvs
	diagnostic_msgs
	geometry_msgs
	rosplan_dispatch_msgs
	rosplan_knowledge_msgs
	rosplan_planning_system
	genmsg
	actionlib
	actionlib_msgs
	tf
	DEPENDS
)

include_directories(
	include
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
	nodes/check_server
	nodes/go_server
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

add_executable(RPGo src/RPGo.cpp)
add_dependencies(RPGo ${catkin_EXPORTED_TARGETS})

add_executable(RPCheck src/RPCheck.cpp)
add_dependencies(RPCheck ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(RPGo ${catkin_LIBRARIES})
target_link_libraries(RPCheck ${catkin_LIBRARIES})