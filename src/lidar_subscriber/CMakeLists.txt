# cmake_minimum_required(VERSION 3.5)
cmake_minimum_required(VERSION 3.5)
project(toyslam)

# Use C++17 as required by your code
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(Eigen3_DIR /usr/include/eigen3/cmake)

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
  visualization_msgs
  xmlrpcpp
  novatel_msgs
  gnss_comm
  nlosexclusion
)

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem serialization)
# find_package(gtsam_points REQUIRED)
find_package(Gflags)
find_package(Glog)


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
    visualization_msgs
    xmlrpcpp
    novatel_msgs
    gnss_comm
    nlosexclusion
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
  ${Boost_INCLUDE_DIRS}
  # ${gtsam_points_INCLUDE_DIRS}
)

# Add GFLAGS and GLOG includes if found
if(Gflags_FOUND)
  include_directories(${GFLAGS_INCLUDE_DIRS})
endif()
if(Glog_FOUND)
  include_directories(${GLOG_INCLUDE_DIRS})
endif()

## Declare C++ executables
add_executable(gnss_imu_sw_node src/gnss_imu_sw_node.cpp)

## Set C++17 for specific targets
set_property(TARGET gnss_imu_sw_node PROPERTY CXX_STANDARD 17)

## Add cmake target dependencies of the executables
add_dependencies(gnss_imu_sw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link for gnss_imu_sw_node
target_link_libraries(gnss_imu_sw_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

# Add GFLAGS and GLOG libraries if found
if(Gflags_FOUND)
  target_link_libraries(gnss_imu_sw_node ${GFLAGS_LIBRARIES})
endif()
if(Glog_FOUND)
  target_link_libraries(gnss_imu_sw_node ${GLOG_LIBRARIES})
endif()

install(TARGETS gnss_imu_sw_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)