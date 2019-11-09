cmake_minimum_required(VERSION 2.8.3)
project(in_hand_collision_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(Eigen3 3.3 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ll4ma_kdl
  ll4ma_collision_wrapper
  roscpp
  rospy
  sensor_msgs
  in_hand_manipulation_msgs
)
set(ccd_DIR $ENV{HOME}/local/lib/ccd/)
set(fcl_DIR $ENV{HOME}/local/lib/cmake/fcl/)

find_package(fcl)

if (fcl_FOUND)
  message("fcl found!")
  set(fcl_INCLUDE_DIRS $ENV{HOME}/local/include)
  set(fcl_LIBRARY_DIRS $ENV{HOME}/local/lib)
  include_directories(${fcl_INCLUDE_DIRS})
endif (fcl_FOUND)

set(ccd_DIR $ENV{HOME}/local/lib/ccd/)
find_package(ccd 2.0 EXACT REQUIRED)
set(ccd_INCLUDE_DIRS $ENV{HOME}/local/include)
set(ccd_LIBRARY_DIRS $ENV{HOME}/local/lib)

catkin_package(
INCLUDE_DIRS include
LIBRARIES hand_env
CATKIN_DEPENDS geometry_msgs ll4ma_kdl ll4ma_collision_wrapper roscpp rospy sensor_msgs
DEPENDS fcl ccd
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/include

)

add_library(hand_env src/scene/hand_env.cpp)
target_link_libraries(hand_env ${catkin_LIBRARIES} fcl)

add_executable(demo_allegro_contact src/demo/allegro_obj_demo.cpp)
target_link_libraries(demo_allegro_contact hand_env ${catkin_LIBRARIES})
