// ROS:
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>
#include <tf/transform_datatypes.h>

#include "in_grasp_manipulation/InGraspSim.h"

namespace in_grasp_sim
{
class robotSim
{
public:
  void init(ros::NodeHandle&);
  void update_robot_state(sensor_msgs::JointState j_cmd);
  void jntCallback(sensor_msgs::JointState j);
  sensor_msgs::JointState curr_js_;
  ros::Publisher j_pub;
  bool new_jcmd=false;
  bool init_jcmd=false;
private:
  sensor_msgs::JointState j_cmd_;
  ros::Subscriber  cmd_sub;

};

class inGraspSim: public robotSim
{
public:
  void update_in_grasp_state();
  bool setup_object(in_grasp_manipulation::InGraspSim::Request&,in_grasp_manipulation::InGraspSim::Response&);
  void init_service(ros::NodeHandle&);
  void publish_state();
  bool setup_obj_frame=false;
  const double mean = 0.0;
  double stddev = 0.0;
  
private:
  
  manipulator_kdl::robotKDL hand_kdl;
  int r_f;
  Eigen::MatrixXd th_T_obj,initial_T_obj;
  ros::Publisher obj_pub;
  ros::ServiceServer in_grasp_init_service;
  geometry_msgs::PoseStamped obj_pose_msg;
  std::default_random_engine generator;
  std::normal_distribution<double> dist;
  Eigen::Matrix4d noise_mat;
  vector<double> noise_arr;
  
};

}
