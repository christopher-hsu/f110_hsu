cmake_minimum_required(VERSION 2.8.3)
project(hsu_chris_scan_matching)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
   CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(scan_matcher src/scan_match.cpp src/transform.cpp src/correspond.cpp src/visualization.cpp)
target_link_libraries(scan_matcher ${catkin_LIBRARIES})