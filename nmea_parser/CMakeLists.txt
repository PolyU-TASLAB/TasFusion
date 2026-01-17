cmake_minimum_required(VERSION 3.5)
project(nmea_parser)

find_package(catkin REQUIRED COMPONENTS
  gnss_comm
  roscpp
  rospy
  std_msgs
)


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
