cmake_minimum_required(VERSION 2.8.3)
project(finger_relocation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
##set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  in_hand_manipulation_msgs
  ll4ma_opt_utils
  #ll4ma_collision_wrapper
  roscpp
  rospy
  in_hand_collision_lib
  in_hand_manipulation
  #ompl_wrapper
)
find_package(PCL 1.7 REQUIRED)

find_package(Pagmo REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

#find_package(ompl REQUIRED)

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

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $ENV{HOME}/pkgs/pagmo_plugins_nonfree/include
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

set(PAGMO_LIBS Pagmo::pagmo  ${Boost_FILESYSTEM_LIBRARIES}  ${Boost_SYSTEM_LIBRARIES}  ${CMAKE_DL_LIBS})

add_library(finger_gait STATIC src/problem/finger_gait.cpp)

target_link_libraries(finger_gait ${PAGMO_LIBS} ${catkin_LIBRARIES})

#add_library(finger_slide STATIC src/problem/finger_slide.cpp src/problem/point_jacobian.cpp)

#target_link_libraries(finger_slide ccd ${PAGMO_LIBS} ${catkin_LIBRARIES} fcl )

add_executable(fg_server src/finger_server.cpp)
target_link_libraries(fg_server ${catkin_LIBRARIES} finger_gait)

#add_library(rrt_finger_gait src/problem/rrt_finger_gait.cpp)
#target_link_libraries(rrt_finger_gait ccd ${catkin_LIBRARIES} fcl ${OMPL_LIBRARIES} ${PAGMO_LIBS})

#add_executable(fg_rrt_server src/fg_rrt_server.cpp)
#target_link_libraries(fg_rrt_server ${catkin_LIBRARIES} rrt_finger_gait ${PAGMO_LIBS})

#add_executable(fs_server src/fs_server.cpp)
#target_link_libraries(fs_server ${catkin_LIBRARIES} finger_slide)
