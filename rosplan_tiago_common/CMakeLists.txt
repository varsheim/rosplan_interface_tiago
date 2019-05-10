cmake_minimum_required(VERSION 2.8.3)
project(rosplan_tiago_common)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  play_motion_msgs
  rospy
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosplan_tiago_common
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs move_base_msgs play_motion_msgs rospy tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()