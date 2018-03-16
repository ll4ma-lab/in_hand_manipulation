cmake_minimum_required(VERSION 2.8.3)
project(hand_task_controllers)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  kdl_typekit
  roscpp
  rospy
  roslib
  geometry_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES hand_control
  CATKIN_DEPENDS kdl_parser kdl_typekit roscpp rospy geometry_msgs sensor_msgs
  DEPENDS system_lib
)

add_library(hand_control src/kdl_controller.cpp)
add_executable(demo_robot_node src/demo_robot_node.cpp)
target_link_libraries(demo_robot_node hand_control ${catkin_LIBRARIES})

#add_executable(jacobian_transpose src/jacobian_transpose.cpp)
#target_link_libraries(jacobian_transpose ${catkin_LIBRARIES})

#add_executable(cartesian_position_controller src/cartesian_position_controller.cpp src/jacobian_controller.cpp)
#target_link_libraries(cartesian_position_controller ${catkin_LIBRARIES})

#add_executable(cartesian_force_controller src/cartesian_force_controller.cpp src/jacobian_controller.cpp)
#target_link_libraries(cartesian_force_controller ${catkin_LIBRARIES})
 
