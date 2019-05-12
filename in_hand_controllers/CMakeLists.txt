cmake_minimum_required(VERSION 2.8.3)
project(in_hand_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ll4ma_kdl
  roscpp
  rospy
  sensor_msgs
  tf
  trajectory_msgs
  #hand_task_controllers
  in_hand_manipulation_msgs
  roslib
  tf_conversions
  hand_services
  grasp_utils
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   TactileGraspControl.srv
#   Service1.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs#   trajectory_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES task_controllers
#  CATKIN_DEPENDS geometry_msgs ll4ma_kdl roscpp rospy sensor_msgs tf trajectory_msgs
#  DEPENDS system_lib
)
#add_library(mpc_framework src/in_grasp/mpc_framework.cpp)
#add_library(fg src/finger_gait/fg.cpp)
#add_library(grasp_contact src/grasp/contact_controller.cpp)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(fg_server src/finger_gait/fg_server.cpp)
#target_link_libraries(fg_server ${catkin_LIBRARIES} fg)
#add_executable(in_grasp_mpc_server_demo src/in_grasp/mpc_server_demo.cpp)
#target_link_libraries(in_grasp_mpc_server_demo mpc_framework ${catkin_LIBRARIES})

#add_executable(in_grasp_mpc_server src/in_grasp/fb_obj_server.cpp)
#target_link_libraries(in_grasp_mpc_server mpc_framework ${catkin_LIBRARIES})

add_library(tactile_grasp_stabilization src/external_controller/tactile_grasp_stabilization.cpp)
target_link_libraries(tactile_grasp_stabilization ${catkin_LIBRARIES})
add_dependencies(tactile_grasp_stabilization ${PROJECT_NAME}_generate_messages_cpp)

add_executable(tactile_grasp_controller src/node/tactile_grasp_controller.cpp)
target_link_libraries(tactile_grasp_controller tactile_grasp_stabilization ${catkin_LIBRARIES})

add_library(eigen_control_interface src/hand_services/eigen_control_interface.cpp)
target_link_libraries(eigen_control_interface ${catkin_LIBRARIES})

add_executable(hand_services_node src/node/hand_services_node.cpp)
target_link_libraries(hand_services_node eigen_control_interface ${catkin_LIBRARIES})
