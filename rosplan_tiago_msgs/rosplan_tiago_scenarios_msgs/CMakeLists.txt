cmake_minimum_required(VERSION 2.8.3)
project(rosplan_tiago_scenarios_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actionlib_msgs
  people_msgs
  geometry_msgs
  message_generation
)

add_action_files(
  DIRECTORY
    action
  FILES
    CheckDishwasher.action
    CheckDoor.action
    CheckLight.action
    GetItem.action
    GetLoad.action
    GiveItem.action
    Go.action
    GoScanning.action
    GoWithAttendance.action
    Greet.action
    HumanApproachDetect.action
    HumanInteract.action
    LeaveLoad.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    people_msgs
    geometry_msgs
    std_msgs
)

catkin_package(

)


