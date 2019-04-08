cmake_minimum_required(VERSION 3.2)
project(in_grasp_manipulation)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wfatal-errors)

OPTION(ENABLE_COLLISION_LIB "Build with COLLISION CHECKING feature" ON)

if(ENABLE_COLLISION_LIB)
  message("Building with collision checking enhancement")
else()
  #DON'T set this to true, that will break the program as there are additional definitions required to enable   collision checking which are handled in the header files. See include/problems/in_grasp_opt_problem.hpp for more details
  add_definitions(-DLL4MA_COLLISION_MODEL=false)
endif(ENABLE_COLLISION_LIB)

## System dependencies are found with CMake's conventions

find_package(Pagmo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  message_generation
  tf
  in_hand_manipulation_msgs
  trajectory_msgs
  in_hand_manipulation
  ll4ma_collision_wrapper
)

find_package(PCL 1.7 REQUIRED)

find_package(Eigen3 REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   InGraspManipulation.srv
   InGraspManipulationCollision.srv
   #InGraspManipulationSd.srv
   InGraspSim.srv
   #RunMPC.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   trajectory_msgs
 )



catkin_package(
  CATKIN_DEPENDS std_msgs roscpp sensor_msgs in_hand_manipulation message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  $ENV{HOME}/pkgs/pagmo_plugins_nonfree/include
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

# collision checking:

set(ccd_DIR $ENV{HOME}/local/lib/ccd/)
set(fcl_DIR $ENV{HOME}/local/lib/cmake/fcl/)

find_package(fcl REQUIRED)
find_package(ccd 2.0 EXACT REQUIRED)

set(ccd_INCLUDE_DIRS $ENV{HOME}/local/include)
set(ccd_LIBRARY_DIRS $ENV{HOME}/local/lib)

message("fcl found!")
set(fcl_INCLUDE_DIRS $ENV{HOME}/local/include)
set(fcl_LIBRARY_DIRS $ENV{HOME}/local/lib)
include_directories(${fcl_INCLUDE_DIRS})
set(COLLISION_LIBS ccd fcl)

# optimization

set(PAGMO_LIBS Pagmo::pagmo  ${Boost_FILESYSTEM_LIBRARIES}  ${Boost_SYSTEM_LIBRARIES}  ${CMAKE_DL_LIBS})


#add_library(rolling STATIC src/problems/in_grasp_rolling.cpp)
#target_link_libraries(rolling ${catkin_LIBRARIES})

#add_library(in_grasp_min_acc_joint STATIC src/problems/in_grasp_joint_smooth.cpp)
#target_link_libraries(in_grasp_min_acc_joint ${catkin_LIBRARIES})

add_library(in_grasp_rr src/problems/in_grasp_rr_const.cpp)
target_link_libraries(in_grasp_rr ${catkin_LIBRARIES})

if(ENABLE_COLLISION_LIB)

add_library(collision_constraints src/problems/collision_constraints.cpp)
target_link_libraries(collision_constraints ${catkin_LIBRARIES} ${COLLISION_LIBS})
endif(ENABLE_COLLISION_LIB)

add_executable(in_grasp_server src/in_grasp_server.cpp)
target_link_libraries(in_grasp_server ${PAGMO_LIBS} ${catkin_LIBRARIES} in_grasp_rr)

if(ENABLE_COLLISION_LIB)
  target_link_libraries(in_grasp_server collision_constraints)
endif(ENABLE_COLLISION_LIB)

#add_library(in_grasp_collision STATIC src/problems/in_grasp_collision.cpp)
#target_link_libraries(in_grasp_collision ${catkin_LIBRARIES})


#add_executable(simulation src/in_grasp_simulation.cpp)
#target_link_libraries(simulation ${catkin_LIBRARIES} in_grasp_sim)

#add_executable(rolling_server src/rolling_server.cpp)
#target_link_libraries(rolling_server ${PAGMO_LIBS} ${catkin_LIBRARIES} rolling)

#add_executable(collision_server src/in_grasp_collision_server.cpp)
#target_link_libraries(collision_server ${PAGMO_LIBS} ${catkin_LIBRARIES} in_grasp_collision)

