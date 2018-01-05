#include <ros/ros.h>
#include "in_hand_manipulation_msgs/RunMPC.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <hand_task_controllers/kdl_controller.h>
#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseArray.h>

#include <ros/package.h>

// Eigen library for vectors and matrices:
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

#include <tf_conversions/tf_kdl.h>
using namespace std;
namespace hand_controller
{
class mpcController : public robotNode
{
public:
  manipulator_kdl::robotKDL* hand_kdl;
  vector<Eigen::VectorXd> j_des;
  vector<KDL::Frame> obj_pose_des;
  KDL::Frame curr_obj_pose,curr_rigid_frame;
  bool got_obj_state;
  
  ros::ServiceServer ss;
  
  void init_controller(tf::TransformListener&);

  void get_object_pose();
  // service callback:
  void set_des_traj(trajectory_msgs::JointTrajectory joint_traj, geometry_msgs::PoseArray obj_pose_arr);

  void publish_joint_pos(Eigen::VectorXd j_pos);
  void run_controller();
  bool srv_call(in_hand_manipulation_msgs::RunMPC::Request& req,in_hand_manipulation_msgs::RunMPC::Response& res);
  double js_gain,obj_gain;
  vector<double> err_;
};
}
