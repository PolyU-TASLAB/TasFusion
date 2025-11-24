cmake_minimum_required(VERSION 3.5)
project(nlosexclusion)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  roscpp
  pcl_conversions
  sensor_msgs
  tf2
  tf2_ros
  pcl_ros
  rospy
  std_msgs
  message_generation
)
add_message_files(
  FILES
  GNSS_Raw.msg
  GNSS_Raw_mf.msg
  GNSS_Raw_Array.msg
  DOP.msg
  Satellite_Info.msg
  exclusionSatNum.msg
  Error.msg
)
generate_messages(
  DEPENDENCIES 
   std_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp message_filters nodelet sensor_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
