cmake_minimum_required(VERSION 3.0.2)
project(robot_fetch_me_a_beer)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_fetch_me_a_beer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
