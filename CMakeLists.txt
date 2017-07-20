cmake_minimum_required(VERSION 2.8.3)
project(in_hand_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  sensor_msgs
  std_msgs
  kdl_parser
  ll4ma_kdl
  biotac_sensors
  optimization_pkg
  allegro_driver
  message_generation
)

add_service_files(
  FILES
  SendPreshapeArr.srv
  BiotacGraspObject.srv
  EnableGraspStability.srv
  UpdateGraspFingers.srv
  DisableGraspStability.srv
  )
generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES in_hand_manipulation
  CATKIN_DEPENDS std_msgs roscpp sensor_msgs
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
