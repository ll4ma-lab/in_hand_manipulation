#include "in_hand_task_controllers/grasp/contact_controller.h"
using namespace std;
namespace hand_controller
{

void mpcController::init_controller()
{
  ss = n_->advertiseService("grasp/control", &hand_controller::mpcController::srv_call, this);

}
// service callback:
void mpcController::set_des_traj(trajectory_msgs::JointTrajectory joint_traj)
{
  j_des.resize(joint_traj.points.size());
  
  for(int i=0;i<joint_traj.points.size();++i)
  {
    Map<Eigen::VectorXd> des_j(&joint_traj.points[i].positions[0],joint_traj.points[i].positions.size());
    j_des[i]=des_j;
  }
}

void mpcController::publish_joint_pos(Eigen::VectorXd j_pos)
{
  joint_cmd_.name=joint_state_.name;
  joint_cmd_.header.stamp=ros::Time::now();
  vector<double> j_arr(j_pos.data(), j_pos.data() + j_pos.size());
  joint_cmd_.position=j_arr;
  joint_pub_.publish(joint_cmd_);
}

bool mpcController::srv_call(in_hand_manipulation_msgs::PrecisionGrasp::Request& req,in_hand_manipulation_msgs::PrecisionGrasp::Response& res)
{
  cerr<<"called service!"<<endl;
  set_des_traj(req.joint_traj);
  contact_prob=req.contact_prob;
  // run controller:
  run_controller();
  // return reached object pose
  updateState();
  
  sensor_msgs::JointState reached_js;
  reached_js=joint_state_;
  return true;
}

}
