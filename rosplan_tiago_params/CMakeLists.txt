cmake_minimum_required(VERSION 2.8.3)
project(rosplan_tiago_params)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf2
  roslib
)

find_package(Boost REQUIRED COMPONENTS)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetLocation.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)

