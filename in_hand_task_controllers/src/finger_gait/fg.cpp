#include "in_hand_task_controllers/finger_gait/fg.h"
using namespace std;
namespace hand_controller
{

void FGController::init_controller()
{
  // get values from ros parameter server:
  // mpc service name:

  ss = n_->advertiseService("in_hand/finger_gait", &hand_controller::FGController::srv_call, this);
  
}

void FGController::publish_joint_pos(Eigen::VectorXd j_pos)
{
  joint_cmd_.name=joint_state_.name;
  joint_cmd_.header.stamp=ros::Time::now();
  vector<double> j_arr(j_pos.data(), j_pos.data() + j_pos.size());
  joint_cmd_.position=j_arr;
  joint_pub_.publish(joint_cmd_);
}

bool FGController::srv_call(in_hand_manipulation_msgs::FingerGaitServo::Request& req, in_hand_manipulation_msgs::FingerGaitServo::Response& res)
{
  cerr<<"called service!"<<endl;
  // update desired joint state:
  vel_thresh=req.vel_thresh;
  finger=req.finger;
  Map<Eigen::VectorXd> d_js(&req.des_joint_st.position[0],req.des_joint_st.position.size());
  des_js=d_js;
  Map<Eigen::VectorXd> i_js(&req.initial_joint_st.position[0],req.initial_joint_st.position.size());

  initial_js=i_js;
  // run controller:
  run_controller();


  // return reached joint_state
  updateState();
  
  //get_object_pose();
  sensor_msgs::JointState reached_js;
  res.reached_js=joint_state_;
  
  return true;
}

}
