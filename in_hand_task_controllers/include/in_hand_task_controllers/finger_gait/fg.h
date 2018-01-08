#include <ros/ros.h>
#include "in_hand_manipulation_msgs/FingerGaitServo.h"
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
class FGController : public robotNode
{
public:
  vector<Eigen::VectorXd> j_des;
  
  ros::ServiceServer ss;
  
  void init_controller();

  void publish_joint_pos(Eigen::VectorXd j_pos);
  void run_controller();
  bool srv_call(in_hand_manipulation_msgs::FingerGaitServo::Request& req,in_hand_manipulation_msgs::FingerGaitServo::Response& res);
  vector<double> err_;
  Eigen::VectorXd des_js,initial_js;
  double vel_thresh;
  int finger;
};
}
