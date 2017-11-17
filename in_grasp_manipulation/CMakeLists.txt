cmake_minimum_required(VERSION 3.2)
project(in_grasp_manipulation)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## System dependencies are found with CMake's conventions
#find_package(Boost 1.61 REQUIRED COMPONENTS system filesystem)
set(BOOST_ROOT /home/bala/local/)

find_package(Boost 1.61 REQUIRED COMPONENTS system filesystem)
find_package(Pagmo REQUIRED)
find_package(ccd REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  #optimization_pkg
  message_generation
  tf
  in_hand_manipulation_msgs
  trajectory_msgs
  ll4ma_planner
)

#find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(Eigen3 REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   InGraspManipulation.srv
   InGraspManipulationSd.srv
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
  CATKIN_DEPENDS std_msgs roscpp sensor_msgs ll4ma_planner message_generation
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /home/bala/pkgs/pagmo_plugins_nonfree/include
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
set(PAGMO_LIBS Pagmo::pagmo  ${Boost_FILESYSTEM_LIBRARIES}  ${Boost_SYSTEM_LIBRARIES}  ${CMAKE_DL_LIBS})

add_library(in_grasp_planner STATIC src/problems/in_grasp.cpp)
target_link_libraries(in_grasp_planner ${catkin_LIBRARIES} ccd)

add_library(in_grasp_min_acc_obj STATIC src/problems/in_grasp_obj_smooth.cpp)
target_link_libraries(in_grasp_min_acc_obj ${catkin_LIBRARIES} ccd)

add_library(in_grasp_min_acc_joint STATIC src/problems/in_grasp_joint_smooth.cpp)
target_link_libraries(in_grasp_min_acc_joint ${catkin_LIBRARIES} ccd)

add_library(in_grasp_sim STATIC src/in_grasp_manipulation/in_grasp_sim.cc)
target_link_libraries(in_grasp_sim ${catkin_LIBRARIES})

add_executable(in_grasp_server src/in_grasp_server.cpp)
target_link_libraries(in_grasp_server ${PAGMO_LIBS} ${catkin_LIBRARIES} in_grasp_planner)

add_executable(in_grasp_mpc_server src/in_grasp_mpc_server.cpp)
target_link_libraries(in_grasp_mpc_server ${PAGMO_LIBS} ${catkin_LIBRARIES} in_grasp_planner)

add_executable(in_grasp_metrics_server src/in_grasp_metrics.cpp)
target_link_libraries(in_grasp_metrics_server ${PAGMO_LIBS} ${catkin_LIBRARIES} in_grasp_planner)

add_executable(in_grasp_obj_server src/in_grasp_min_acc_server.cpp)
target_link_libraries(in_grasp_obj_server ${PAGMO_LIBS} ${catkin_LIBRARIES} in_grasp_min_acc_obj)

add_executable(in_grasp_joint_server src/in_grasp_min_acc_server.cpp)
target_link_libraries(in_grasp_joint_server ${PAGMO_LIBS} ${catkin_LIBRARIES} in_grasp_min_acc_joint)

add_executable(simulation src/in_grasp_simulation.cpp)
target_link_libraries(simulation ${catkin_LIBRARIES} in_grasp_sim)
#add_executable(one_step_mpc src/controllers/one_step_mpc.cpp)
#target_link_libraries(one_step_mpc ${catkin_LIBRARIES})
#add_library(in_grasp_sd_planner STATIC src/problems/in_grasp_gait.cpp)
#target_link_libraries(in_grasp_sd_planner  ${PAGMO_LIBS} ${catkin_LIBRARIES} ccd)

#add_library(in_grasp_rr_const STATIC src/problems/in_grasp_rr_const.cpp)
#target_link_libraries(in_grasp_rr_const  ${PAGMO_LIBS} ${catkin_LIBRARIES} ccd)

#add_library(in_grasp_gait_traj_planner STATIC src/problems/in_grasp_gait_traj.cpp)
#target_link_libraries(in_grasp_gait_traj_planner  ${PAGMO_LIBS} ${catkin_LIBRARIES} ccd)

#add_executable(in_grasp_sd_server src/in_grasp_sd_server.cpp)
#target_link_libraries(in_grasp_sd_server ${PAGMO_LIBS}  ${catkin_LIBRARIES} in_grasp_sd_planner)

#add_executable(in_grasp_reachable_server src/in_grasp_dpt_server.cpp)
#target_link_libraries(in_grasp_reachable_server ${PAGMO_LIBS}  ${catkin_LIBRARIES} in_grasp_rr_const)

#add_executable(in_grasp_gait_traj_server src/in_grasp_gait_traj_server.cpp)
#target_link_libraries(in_grasp_gait_traj_server ${PAGMO_LIBS}  ${catkin_LIBRARIES} in_grasp_gait_traj_planner)

