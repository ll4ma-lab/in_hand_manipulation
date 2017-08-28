cmake_minimum_required(VERSION 3.2)
project(in_grasp_manipulation)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  roslib
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  ll4ma_kdl
  optimization_pkg
  message_generation
  trajectory_msgs
  pcl_mesh_tools
  tf
)

set(BOOST_ROOT /home/bala/local/)
## System dependencies are found with CMake's conventions
find_package(Boost 1.61 REQUIRED COMPONENTS system filesystem)
find_package(Pagmo REQUIRED)
find_package(ccd REQUIRED)
#find_package(Eigen REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   InGraspManipulation.srv
   InGraspManipulationSd.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   trajectory_msgs
   geometry_msgs
   sensor_msgs
 )

catkin_package(
  
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /home/bala/pkgs/pagmo_plugins_nonfree/include
  ${Boost_INCLUDE_DIRS}
  )
set(PAGMO_LIBS Pagmo::pagmo  ${Boost_FILESYSTEM_LIBRARIES}  ${Boost_SYSTEM_LIBRARIES}  ${CMAKE_DL_LIBS})

add_library(in_grasp_planner STATIC src/problems/in_grasp.cpp)
target_link_libraries(in_grasp_planner ccd ${catkin_LIBRARIES})
add_executable(in_grasp_server src/in_grasp_server.cpp)
target_link_libraries(in_grasp_server ${catkin_LIBRARIES} in_grasp_planner)
