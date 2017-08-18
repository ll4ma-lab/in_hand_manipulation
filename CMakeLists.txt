cmake_minimum_required(VERSION 3.2)
project(in_hand_manipulation)

add_compile_options(-std=c++11)


set(BOOST_ROOT /home/bala/local/)
## System dependencies are found with CMake's conventions
find_package(Boost 1.61 REQUIRED COMPONENTS system filesystem)
find_package(Pagmo REQUIRED)
find_package(ccd REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  pcl_ros
  sensor_msgs
  std_msgs
  kdl_parser
  ll4ma_kdl
  biotac_sensors
  optimization_pkg
  allegro_driver
  message_generation
  pcl_mesh_tools
  trajectory_msgs
  tf
  in_grasp_manipulation
)
find_package(PCL 1.7 REQUIRED)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  SendPreshapeArr.srv
  BiotacGraspObject.srv
  EnableGraspStability.srv
  UpdateGraspFingers.srv
  DisableGraspStability.srv
  PlanFingerGait.srv
  GetHandIK.srv
  )
generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  )

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES in_hand_manipulation
  CATKIN_DEPENDS std_msgs roscpp sensor_msgs
  #DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  /home/bala/pkgs/pagmo_plugins_nonfree/include
  ${Boost_INCLUDE_DIRS}
  )
set(PAGMO_LIBS Pagmo::pagmo  ${Boost_FILESYSTEM_LIBRARIES}  ${Boost_SYSTEM_LIBRARIES}  ${CMAKE_DL_LIBS})

add_library(finger_gaiting_planner STATIC src/problems/finger_gaiting_fns.cpp)

add_library(hand_ik_fns STATIC src/problems/hand_ik_fns.cpp)

target_link_libraries(finger_gaiting_planner ${catkin_LIBRARIES} ccd )
target_link_libraries(hand_ik_fns ${catkin_LIBRARIES} ccd )
add_executable(finger_gaiting_server src/finger_gaiting_server.cpp)
target_link_libraries(finger_gaiting_server ${PAGMO_LIBS}  ${catkin_LIBRARIES} finger_gaiting_planner)

add_executable(hand_ik_server src/hand_ik_server.cpp)
target_link_libraries(hand_ik_server  ${PAGMO_LIBS}  ${catkin_LIBRARIES} hand_ik_fns)
