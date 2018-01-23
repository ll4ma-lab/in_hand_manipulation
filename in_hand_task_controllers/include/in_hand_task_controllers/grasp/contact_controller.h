#include <ros/ros.h>
#include "in_hand_manipulation_msgs/PrecisionGrasp.h"
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
  vector<Eigen::VectorXd> j_des;
  
  ros::ServiceServer ss;
  
  void init_controller();

  // service callback:
  void set_des_traj(trajectory_msgs::JointTrajectory joint_traj);

  void publish_joint_pos(Eigen::VectorXd j_pos);
  void run_controller();
  bool srv_call(in_hand_manipulation_msgs::PrecisionGrasp::Request& req,in_hand_manipulation_msgs::PrecisionGrasp::Response& res);
  
  double js_gain;
  double contact_prob;
  vector<double> err_;
  // biotac subscriber:
  
};
}
