cmake_minimum_required(VERSION 2.8.3)
project(grasp_gaits)
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
  visualization_msgs
  ll4ma_kdl
  std_msgs
  geometry_msgs
  message_generation
  trajectory_msgs
  pcl_mesh_tools
  optimization_pkg
  in_hand_manipulation
)

find_package(Pagmo REQUIRED)

set(ccd_DIR $ENV{HOME}/local/lib/ccd/)
find_package(ccd 2.0 EXACT REQUIRED)
set(ccd_INCLUDE_DIRS $ENV{HOME}/local/include)
set(ccd_LIBRARY_DIRS $ENV{HOME}/local/lib)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Ellipse.msg
   Grasp.msg
   GaitNode.msg
   GaitPlan.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetEllipse.srv
  RelocateFinger.srv
  RelocateFingerKin.srv
  MoveObj.srv
  GetGaitPlan.srv
  RelocateObj.srv
  RelocateObjSd.srv
  ComputeGD.srv
  ComputeGDRaw.srv
  UpdateObj.srv
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
   grasp_gaits
   trajectory_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES grasp_gaits
  CATKIN_DEPENDS message_runtime
#  DEPENDS s
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $ENV{HOME}/pkgs/pagmo_plugins_nonfree/include
  ${Boost_INCLUDE_DIRS}
)

set(PAGMO_LIBS Pagmo::pagmo  ${Boost_FILESYSTEM_LIBRARIES}  ${Boost_SYSTEM_LIBRARIES}  ${CMAKE_DL_LIBS})

add_library(finger_gait STATIC src/problem/finger_gait.cpp)
target_link_libraries(finger_gait ccd ${PAGMO_LIBS} ${catkin_LIBRARIES})

add_library(allegro_env STATIC src/scene/allegro_env.cpp)
target_link_libraries(allegro_env ccd ${catkin_LIBRARIES})

add_executable(fg_server src/fg_server.cpp)
target_link_libraries(fg_server finger_gait allegro_env ccd ${PAGMO_LIBS} ${catkin_LIBRARIES})
