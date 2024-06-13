cmake_minimum_required(VERSION 3.0.2)
project(robot_fetch_me_a_beer)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  DetectionControl.srv  # add service definition file here
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS srv
#  LIBRARIES robot_fetch_me_a_beer
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
#include
  ${catkin_INCLUDE_DIRS}
)