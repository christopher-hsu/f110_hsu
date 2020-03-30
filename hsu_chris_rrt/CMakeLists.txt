cmake_minimum_required(VERSION 2.8.3)
project(hsu_chris_rrt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set (CMAKE_CXX_FLAGS -O3)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rrt
 CATKIN_DEPENDS ackermann_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs tf
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(hsu_rrt_node node/rrt_node.cpp src/rrt.cpp)

target_link_libraries(hsu_rrt_node
  ${catkin_LIBRARIES}
)