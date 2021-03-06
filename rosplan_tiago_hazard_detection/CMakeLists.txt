cmake_minimum_required(VERSION 2.8.3)
project(rosplan_tiago_hazard_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	diagnostic_msgs
	geometry_msgs
	roscpp
	rosplan_dispatch_msgs
	rosplan_knowledge_msgs
	rosplan_tiago_scenarios_msgs
	rosplan_tiago_core_msgs
	rosplan_planning_system
	rosplan_tiago_params
	rosplan_tiago_common
	rospy
	std_msgs
	std_srvs
	tf
	actionlib
	actionlib_msgs
	smach
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_python_setup()

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
	rosplan_tiago_scenarios_msgs
	rosplan_tiago_core_msgs
	rosplan_planning_system
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
	nodes/check_light_server
	nodes/check_door_server
	nodes/check_dishwasher_server
	nodes/go_server
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

add_executable(RPGo src/RPGo.cpp)
add_dependencies(RPGo ${catkin_EXPORTED_TARGETS})

add_executable(RPCheckLight src/RPCheckLight.cpp)
add_dependencies(RPCheckLight ${catkin_EXPORTED_TARGETS})

add_executable(RPCheckDoor src/RPCheckDoor.cpp)
add_dependencies(RPCheckDoor ${catkin_EXPORTED_TARGETS})

add_executable(RPCheckDishwasher src/RPCheckDishwasher.cpp)
add_dependencies(RPCheckDishwasher ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(RPGo ${catkin_LIBRARIES})
target_link_libraries(RPCheckDoor ${catkin_LIBRARIES})
target_link_libraries(RPCheckLight ${catkin_LIBRARIES})
target_link_libraries(RPCheckDishwasher ${catkin_LIBRARIES})