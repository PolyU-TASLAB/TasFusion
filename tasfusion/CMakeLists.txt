cmake_minimum_required(VERSION 3.5)
project(tasfusion)

# Use C++17 as required by codes
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "Release flags: ${CMAKE_CXX_FLAGS_RELEASE}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  tf2_ros
  nav_msgs
  geometry_msgs
  xmlrpcpp
  novatel_msgs
  gnss_comm
  nlosexclusion
  visualization_msgs
)

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

## Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    tf
    tf2_ros 
    nav_msgs 
    geometry_msgs 
    xmlrpcpp
    novatel_msgs
    gnss_comm
    nlosexclusion
    visualization_msgs
  DEPENDS 
    EIGEN3 
    CERES 
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(gnss_imu_sw_node src/gnss_imu_sw_node.cpp)

## Add cmake target dependencies of the executables
add_dependencies(gnss_imu_sw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link for gnss_imu_sw_node
target_link_libraries(gnss_imu_sw_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

## Install targets
install(TARGETS gnss_imu_sw_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)