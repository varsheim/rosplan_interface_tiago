cmake_minimum_required(VERSION 2.8.3)
project(rosplan_tiago_transportation_attendant)

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
	people_msgs
	rosplan_planning_system
	rosplan_tiago_params
	rosplan_tiago_common
	rospy
	std_msgs
	std_srvs
	tf
	rosplan_tiago_scenarios_msgs
	rosplan_tiago_core_msgs
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
	rosplan_planning_system
	rosplan_tiago_scenarios_msgs
	rosplan_tiago_core_msgs
	actionlib
	actionlib_msgs
	people_msgs
	tf
	DEPENDS
)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
        nodes/go_with_attendace_server
        nodes/leave_load_server
        nodes/get_load_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

add_executable(RPGoWithAttendance src/RPGoWithAttendance.cpp)
add_dependencies(RPGoWithAttendance ${catkin_EXPORTED_TARGETS})

add_executable(RPGetLoad src/RPGetLoad.cpp)
add_dependencies(RPGetLoad ${catkin_EXPORTED_TARGETS})

add_executable(RPLeaveLoad src/RPLeaveLoad.cpp)
add_dependencies(RPLeaveLoad ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(RPGoWithAttendance ${catkin_LIBRARIES})
target_link_libraries(RPGetLoad ${catkin_LIBRARIES})
target_link_libraries(RPLeaveLoad ${catkin_LIBRARIES})