#include <hand_services/external_controller/tactile_grasp_stabilization.h>
namespace hand_task_controller
{
bool tactileGraspStabilization::configureHook()
{
  if(!inHandController::configureHook())
    return false;
  // read control parameters from node
  nh_.getParam("update_srv_name",srv_name);
  update_control_req=nh_.advertiseService(srv_name,&tactileGraspStabilization::update_params,this);
  // initialize tactile data:
  
  return true;
}

bool tactileControl(Eigen::VectorXd &j_cmd)
{
  // controller law:

  return true;
}
bool tactileGraspStabilization::get_control_cmd(Eigen::VectorXd &q_cmd, Eigen::VectorXd &qd_cmd)
{
  return tactileControl(q_cmd);
  
}
    
bool tactileGraspStabilization::update_params(hand_services::TactileGraspControl::Request &req, hand_services::TactileGraspControl::Response &res)
{
  // get number of fingers

  // get eigen amplitudes

  // 
}

}

