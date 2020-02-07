#include <in_hand_controllers/external_controller/tactile_friction_estimation.h>
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
  update_control_req_=nh_.advertiseService(srv_name,&tactileController::estimation_srv,this);

    // read subscriber topic for tactile force sensing
  string tactile_sub_topic;
  success &= nh_.getParam("tactile_force_topic",tactile_sub_topic);
  string tactile_raw_sub_topic;
  success &= nh_.getParam("tactile_raw_topic",tactile_raw_sub_topic);
  

  success &= nh_.getParam("cmd_timeout",cmd_timeout_);

  success &= nh_.getParam("kinematic_slip_thresh",kin_slip_thresh_);
  if(!success)
  {
    return false;
  }

  // create subscriber
  tactile_sub_=nh_.subscribe<biotac_sensors::BioTacForce>(tactile_sub_topic,1,&tactileController::tactile_msg_callback,this);

  
  // create biotac raw subscriber

  tactile_raw_sub_ =  nh_.subscribe<biotac_sensors::BioTacTareHand>(tactile_raw_sub_topic,1,&tactileController::tactile_raw_msg_callback,this);

  return true;
}

bool tactileController::tactileSlip()
{
  // get force in robot palm frame

  // check if within limit
  for(int i=0;i<f_idx.size();++i)
  {
    //if(f_idx[i]==0)
    {
    Eigen::MatrixXd T;
    robot_interface_->kdl_->getFK(f_idx[i],q_,T);
    //cerr<<tac_forces.size()<<endl;
    //cerr<<i<<endl;
    //cerr<<tac_forces[i]<<endl;

    Eigen::VectorXd palm_f=T.block(0,0,3,3)*tac_forces[f_idx[i]];
    // normalize vector:
    palm_f[0]=palm_f[0]+0.00000001;
    palm_f.normalize();
    if(palm_f[1]>deviation_force_)
    {
      ROS_INFO("Slip detected %d",f_idx[i]);
      return true;
    }
    }
  }
  // return true if outside limit
  return false;
}
bool tactileController::tactileControl(Eigen::VectorXd &j_cmd)
{
  /*
  // controller law:
  // Check if force on fingertips are perpendular to the common axis, for allegro it is the y axis.
  
  // if force is not perpendicular, increase joint position command
  if(tactileSlip())
  {
    Eigen::VectorXd q_dot_cmd(16);
    q_dot_cmd.setZero();

    Eigen::VectorXd alpha_dot,theta_dot;
    alpha_dot.resize(eigen_grasp_.eg_size);
    
    for(int i=0;i<eigen_grasp_.eg_size;++i)
    {
      alpha_dot[i]=grasp_ampl_[i]-eigen_grasp_.a_range_[0];
    }

    // get joint command:
    Eigen::MatrixXd J;
    eigen_grasp_.get_jacobian(J);
    theta_dot=J*alpha_dot;
    Eigen::VectorXd q_current_cmd=q_servo_cmd_;// 
    for(int i=0;i<f_idx.size();++i)
    {
      
      Eigen::VectorXd q_dot_f=theta_dot.segment(4*f_idx[i],4);
      q_dot_f=stiffness_delta_*q_dot_f/q_dot_f.lpNorm<Eigen::Infinity>();
      if(f_idx[i]<3)
      {
        q_dot_f=0.3*q_dot_f;
      }
      q_dot_cmd.segment(4*f_idx[i],4)=q_dot_f;
   
    }

    q_current_cmd+=q_dot_cmd;
    j_cmd=q_current_cmd;
    q_servo_cmd_=q_current_cmd;

    return true;
  }
  */
  return false;

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
  cerr<<"returning false"<<endl;
  return false;
    
}

bool tactileController::estimate_friction(const int &finger, const Eigen::Vector3d &force_pt, const Eigen::Vector3d &force_direction, const double &force_mag, Eigen::Vector3d &slip_force)
{

  // create a timeout:
  bool timeout=false;
  ros::Time begin = ros::Time::now();
  ros::Time current = ros::Time::now();
  
  robot_interface::ControlMode cm=robot_interface::ControlMode::POS;// 0==position

  
  // hold object rigid

  // tactile force:

  
  // TODO: test with one finger and then add stability command for other fingers
  // read force from fingertip
  Eigen::VectorXd current_force=tac_forces[finger];

  // access kdl chain for fingertip
  Eigen::MatrixXd J;

  // get jacobian to force pt
  log("Estimating Friction");
  Eigen::VectorXd new_cmd_pt=force_pt;//+force_direction;
  Eigen::VectorXd old_cmd=last_jcmd_;
  //cerr<<"Finger: "<<finger<<endl;
  while(!kinematicSlip(finger,force_direction) && !timeout)
  {
    //log("slip not detected yet");
    ros::spinOnce();
    current = ros::Time::now();
    robot_interface_->kdl_->getJacobian(finger,q_,J);
    Eigen::VectorXd theta_dot=J.block(0,0,3,J.cols()).transpose()*new_cmd_pt;
    //cerr<<theta_dot.transpose()<<endl;
    Eigen::VectorXd theta_cmd=old_cmd;
    theta_cmd.segment(finger*4,4)+=theta_dot.segment(finger*4,4);
    //cerr<<theta_dot.segment(finger*4,4).transpose()<<endl;
    robot_interface_->publishRealRobot(theta_cmd,cm);
    new_cmd_pt+=force_direction*force_mag;
    rate_.sleep();
    // check timer and update
    if(current.toSec()-begin.toSec()>cmd_timeout_)
    {
      log("Timed-out");
      
      timeout=true;
      slip_force.resize(3);
      slip_force.setZero();
      return false;

    }
    last_jcmd_=theta_cmd;
    current_force=tac_forces[finger];


  }
  // change point by force direction with magnitude
  // get jacobian and multiply by velocity vector

  //
  // transform force to palm frame:
  Eigen::MatrixXd T;
  robot_interface_->kdl_->getFK(finger,q_,T);
  slip_force=T.block(0,0,3,3)*current_force;
  //slip_force=current_force;

  return true;
}

bool tactileController::estimation_srv(in_hand_controllers::TactileFrictionEstimation::Request &req, in_hand_controllers::TactileFrictionEstimation::Response &res)
{
  // get finger to estimate friction:
  int f=req.f_idx;
  
  // get joint position state that was last commanded
  last_jcmd_.resize(req.last_jcmd.position.size());
  for(int i=0;i<req.last_jcmd.position.size();++i)
  {
    last_jcmd_[i]=req.last_jcmd.position[i];
  }

  // estimate friction:
  Eigen::Vector3d f_slip;
  Eigen::Vector3d force_direction,force_pt;
  double force_mag=req.force_mag;
  force_direction[0]=req.force_direction[0];
  force_direction[1]=req.force_direction[1];
  force_direction[2]=req.force_direction[2];
  force_direction.normalize();
  force_pt[0]=req.force_pt[0];
  force_pt[1]=req.force_pt[1];
  force_pt[2]=req.force_pt[2];

  res.success=estimate_friction(f,force_pt,force_direction,force_mag,f_slip);
  res.slip_force.resize(3);
  res.slip_force[0]=f_slip[0];
  res.slip_force[1]=f_slip[1];
  res.slip_force[2]=f_slip[2];
 
  q_servo_cmd_=last_jcmd_;
  
  res.last_jcmd=req.last_jcmd;
  for(int i=0;i<last_jcmd_.size();++i)
  {
    res.last_jcmd.position[i]=last_jcmd_[i];
  }
  res.biotac_tare_msg = tactile_raw_msg_;
  //res.last_jcmd.position=last_jcmd_;
  log("Friction estimated");
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

void tactileController::tactile_raw_msg_callback(biotac_sensors::BioTacTareHand msg)
{

  // read messages from tactile sensor
  
  tactile_raw_msg_ = msg;

  
}

}

