#include<hand_services/external_controller/relaxed_rigidity.h>

namespace hand_task_controller
{

bool relaxedRigidityGrasp::configureHook()
{
  if(!inHandController::configureHook())
    return false;

  rr_parameters_set_=false;

  // read parameters
  // gains:
  kp_.resize(4);
  kd_.resize(4);

  getGainsFromParamServer("rr/kp",nh_,kp_);
  getGainsFromParamServer("rr/kd",nh_,kd_);

  string srv_name;
  nh_.getParam("update_srv_name",srv_name);
  update_control_req_=nh_.advertiseService(srv_name,&relaxedRigidityGrasp::update_relaxed_rigidity_params,this);

  grasp_kp=0.0;
  nh_.getParam("rr/init_grasp_kp",grasp_kp);
  return true;
}
bool relaxedRigidityGrasp::update_relaxed_rigidity_params(hand_services::updateRelaxedRigidityParams::Request &req, hand_services::updateRelaxedRigidityParams::Response &res)
{
  cerr<<"Relaxed rigidity grasp parameters updated!"<<endl;
  //get fingers
  force_dirs_.resize(4);
  f_idx.resize(req.fingers.size());
  for(int i=0;i<f_idx.size();++i)
  {
    f_idx[i]=req.fingers[i];
  }

  // get joint configuration:
  Eigen::Map<Eigen::VectorXd> q_init(&req.grasp_js.position[0],req.grasp_js.position.size());
  
  // get pose offsets:
  rigid_f=req.rigid_finger;

  Eigen::VectorXd th_pose;
  robot_interface_->kdl_->getFK(3,q_init.segment(3*4,4),th_pose);

  Eigen::VectorXd in_pose;
  robot_interface_->kdl_->getFK(0,q_init.segment(0*4,4),in_pose);

  // position force
  
  for(int i=0;i<f_idx.size();++i)
  {
    if(f_idx[i]!=3)
    {
      Eigen::VectorXd f_pose;
      robot_interface_->kdl_->getFK(f_idx[i],q_init.segment(f_idx[i]*4,4),f_pose);
      force_dirs_[f_idx[i]]=th_pose.head(3)-f_pose.head(3);
    }
    else
    {
      Eigen::VectorXd f_pose;
      robot_interface_->kdl_->getFK(f_idx[i],q_init.segment(f_idx[i]*4,4),f_pose);
      force_dirs_[f_idx[i]]=in_pose.head(3)-f_pose.head(3);
      
    }   
  }
  init_grasp_=q_init;
  rr_parameters_set_=true;
  return true;
}


bool relaxedRigidityGrasp::get_control_cmd(Eigen::VectorXd &q, Eigen::VectorXd &qd)
{
  // call relaxed rigidity control
  relaxedRigidityControl(q,qd);
}

bool relaxedRigidityGrasp::relaxedRigidityControl(Eigen::VectorXd &j_cmd, Eigen::VectorXd &jdot_cmd)
{
  assert(rr_parameters_set_);
  while(robot_state_received_<1)
  {
    ros::spinOnce();
  }
  Eigen::VectorXd th_pose;
  robot_interface_->kdl_->getFK(3,q_.segment(3*4,4),th_pose);

  for(int i=0;i<f_idx.size();++i)
  {
    Eigen::VectorXd f_pose;
    // weigh the force by the tactile force value:
    robot_interface_->kdl_->getFK(f_idx[i],q_.segment(f_idx[i]*4,4),f_pose);
    Eigen::Vector3d force_vec=force_dirs_[f_idx[i]]-(th_pose.head(3)-f_pose.head(3));
    
    // jacobian sandwitch for control:
    Eigen::Matrix3d kp_mat;
    kp_mat.setIdentity();
    kp_mat=kp_[f_idx[i]]*kp_mat;
    Eigen::MatrixXd J;
    robot_interface_->kdl_->getJacobian(f_idx[i],q_.segment(f_idx[i]*4,4),J);
    J=J.block(0,0,3,J.cols());
    Eigen::MatrixXd kp_j;
    kp_j=(J.transpose()*kp_mat)*J;
    Eigen::Vector3d delta_force=-10.0*(force_vec);
    // convert delta force to joint space velocity:
    
    if(f_idx[i]==0)
    {
      cerr<<delta_force.transpose()<<endl;    
      j_cmd.segment(f_idx[i]*4,4)=init_grasp_.segment(f_idx[i]*4,4).transpose()+delta_force.transpose()*J.block(0,0,3, J.cols())*kp_j - kd_[f_idx[i]] * q_dot_.segment(f_idx[i]*4,4).transpose();
    }
    else
    {
      j_cmd.segment(f_idx[i]*4,4)=init_grasp_.segment(f_idx[i]*4,4).transpose();
    }
    //jdot_cmd.segment(f_idx[i]*4,4)=delta_force.transpose()*J.block(0,0,3, J.cols());
  }
  //j_cmd=init_grasp_;

  //j_cmd+=grasp_kp*(init_grasp_-q_);
  // The idea is to follow the thumb pose with the other fingers:
  return true;
}


}
