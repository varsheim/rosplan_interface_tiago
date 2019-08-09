cmake_minimum_required(VERSION 2.8.3)
project(rosplan_tiago_common)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  move_base_msgs
  play_motion_msgs
  rospy
  tf
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

