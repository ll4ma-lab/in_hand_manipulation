#include <in_hand_controllers/external_controller/tactile_impedance_grasp.h>
namespace hand_task_controller
{

tactileController::tactileController(string ns, robot_interface::RobotInterface *robot_interface, double loop_rate): inHandController(ns,robot_interface,loop_rate)
{

}

bool tactileController::configureHook()
{
  if(!inHandController::configureHook())
    return false;
  //log(''
  bool success = true;
  // read control parameters from node
  // initialize tactile data:
  string srv_name;
  success &= nh_.getParam("update_srv_name",srv_name);
  update_control_req_=nh_.advertiseService(srv_name,&tactileController::update_control_srv,this);

    // read subscriber topic for tactile force sensing
  string tactile_sub_topic, surface_norm_sub_topic;
  success &= nh_.getParam("tactile_force_topic",tactile_sub_topic);
  success &= nh_.getParam("surface_norm_topic",surface_norm_sub_topic);
  

  success &= nh_.getParam("cmd_timeout",cmd_timeout_);

  success &= nh_.getParam("kinematic_slip_thresh",kin_slip_thresh_);
  if(!success)
  {
    log("Failed to initialize tactile controller");
    return false;
  }

  // create subscriber
  tactile_sub_=nh_.subscribe<biotac_sensors::BioTacForce>(tactile_sub_topic,1,&tactileController::tactile_msg_callback,this);
  surface_sub_=nh_.subscribe<in_hand_manipulation_msgs::SurfaceNormal>(surface_norm_sub_topic,1,&tactileController::surface_normal_callback,this);
  

  return true;
}

bool tactileController::tactileSlip(const int &finger)
{
  // get force in robot palm frame
  Eigen::MatrixXd T;
  robot_interface_->kdl_->getFK(finger,q_,T);
  Eigen::Vector3d palm_f=T.block(0,0,3,3)*tac_forces[finger];

  
  palm_f.normalize();
  // compare force with surface normal
  if(acos(palm_f.dot(surface_normal_[finger]))>atan(mu_))
  {
    return true;
  }
  
  return false;
}
bool tactileController::tactileControl(Eigen::VectorXd &j_cmd)
{

  // this is only increasing force, how to also decrease force?
  
  // get current joint position

  // check if fingers are slipping


  j_cmd=last_jcmd_;
  for(int i=0;i<f_idx.size();++i)
  {
    f_mag_[f_idx[i]]=alpha_[f_idx[i]]*f_idx[i];
    if(tactileSlip(f_idx[i]))
    {
      f_mag_[f_idx[i]]+=(1-alpha_[f_idx[i]]);    
    }
    // get joint velocity command:
    Eigen::MatrixXd J;
    robot_interface_->kdl_->getJacobian(f_idx[i],q_,J);


    // increase force along??
    Eigen::VectorXd theta_dot;//=J.block(0,0,3,J.cols()).transpose()*new_cmd_pt;
    theta_dot=f_mag_[f_idx[i]]*theta_dot;
    j_cmd.segment(f_idx[i],4)+=theta_dot;

  }

  
  return true;
  
}
bool tactileController::kinematicSlip(const int &finger, const Eigen::Vector3d &slip_dir)
{
  if(old_ftip_pose_.size()!=3)
  {
    Eigen::VectorXd f_pose;
    // check fk of current fingertip pose
    robot_interface_->kdl_->getFK(finger,q_,f_pose,true);
    old_ftip_pose_=f_pose.head(3);
    return false;

  }

  // check deviation
  Eigen::VectorXd f_pose;
  // check fk of current fingertip pose
  robot_interface_->kdl_->getFK(finger,q_,f_pose,true);
  Eigen::Vector3d diff=f_pose.head(3)-old_ftip_pose_;
  Eigen::Matrix3d w_mat=slip_dir.asDiagonal();
  double dt_rate=rate_.cycleTime().toSec();
  //cerr<<sqrt(diff.transpose()*w_mat*diff)/dt_rate<<endl;
  
  if((sqrt(diff.transpose()*w_mat*diff)/dt_rate)>kin_slip_thresh_)
  {
    log("Slip detected!!!");
    cerr<<sqrt(diff.transpose()*w_mat*diff)/dt_rate<<endl;
    old_ftip_pose_=Eigen::VectorXd();
    return true;
  }
  old_ftip_pose_=f_pose.head(3);
  return false;
}

bool tactileController::get_control_cmd(Eigen::VectorXd &q_cmd, Eigen::VectorXd &qd_cmd)
{
  // get control command:
  // run stabilization controller:
  if(got_surface_normal_ && tactile_msg_)
  {
    // get tactile impedance command
    return tactileControl(q_cmd);
  }
  else
  {
  return false;
  }
    
}

bool tactileController::update_control_srv(in_hand_controllers::TactileImpedanceGrasp::Request &req, in_hand_controllers::TactileImpedanceGrasp::Response &res)
{

  // get fingers to perform stabilization with
  f_idx=vector<int>(std::begin(req.f_idx),std::end(req.f_idx));

  mu_=req.friction_coeff;
  
  
  // get joint position state that was last commanded
  last_jcmd_.resize(req.last_jcmd.position.size());
  for(int i=0;i<req.last_jcmd.position.size();++i)
  {
    last_jcmd_[i]=req.last_jcmd.position[i];
  }

  res.success=true;
 
  q_servo_cmd_=last_jcmd_;
  
        
  //res.last_jcmd.position=last_jcmd_;
  log("Impedance controller service updated parameters");
  return true;
                   
  
}

void tactileController::tactile_msg_callback(biotac_sensors::BioTacForce msg)
{

  // read messages from tactile sensor
  
  if(f_idx.size()>msg.forces.size())
  {
    log("Missing Biotac sensor data for all fingertips");
    tactile_msg_=false;
  }
  else
  {
    tac_forces.resize(msg.forces.size());
    tac_contacts.resize(msg.forces.size());

    for (int i=0; i<msg.forces.size();++i)
    {
      //int k=f_idx[i];
      tac_forces[i].resize(3);
      tac_forces[i][0]=msg.forces[i].wrench.force.x;
      tac_forces[i][1]=msg.forces[i].wrench.force.y;
      tac_forces[i][2]=msg.forces[i].wrench.force.z;
      tac_contacts[i]=0;//msg.contact.data[i];
    }
    tactile_msg_=true;
      
  }

  
}
void tactileController::surface_normal_callback(in_hand_manipulation_msgs::SurfaceNormal msg)
{
  // read surface normal at contact points
  // the surface normal is in robot palm frame, same frame as the fingertip kdl base frame
  surface_normal_.resize(msg.surface_normal_arr.size());
  for (int i=0; i<surface_normal_.size();++i)
  {
    surface_normal_[i][0]=msg.surface_normal_arr[i].pose.position.x;
    surface_normal_[i][1]=msg.surface_normal_arr[i].pose.position.y;
    surface_normal_[i][2]=msg.surface_normal_arr[i].pose.position.z;
            
  }
  got_surface_normal_=true;
}


}

