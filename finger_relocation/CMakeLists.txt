cmake_minimum_required(VERSION 2.8.3)
project(finger_relocation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  in_hand_manipulation_msgs
  optimization_pkg
  pcl_mesh_tools
  roscpp
  rospy
  in_hand_collision_lib
)

find_package(fcl)

if (fcl_FOUND)
  message("fcl found!")
  include_directories(${fcl_INCLUDE_DIRS})
endif (fcl_FOUND)

find_package(Pagmo REQUIRED)

set(ccd_DIR $ENV{HOME}/local/lib/ccd/)
find_package(ccd 2.0 EXACT REQUIRED)
set(ccd_INCLUDE_DIRS $ENV{HOME}/local/include)
set(ccd_LIBRARY_DIRS $ENV{HOME}/local/lib)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $ENV{HOME}/pkgs/pagmo_plugins_nonfree/include
  ${Boost_INCLUDE_DIRS}
)

set(PAGMO_LIBS Pagmo::pagmo  ${Boost_FILESYSTEM_LIBRARIES}  ${Boost_SYSTEM_LIBRARIES}  ${CMAKE_DL_LIBS})

add_library(finger_gait STATIC src/problem/finger_gait.cpp)

target_link_libraries(finger_gait ccd ${PAGMO_LIBS} ${catkin_LIBRARIES} fcl)

