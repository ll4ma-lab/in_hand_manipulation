#include <in_hand_controllers/external_controller/tactile_friction_estimation.h>
namespace hand_task_controller
{

tactileController::tactileController(string ns, robot_interface::RobotInterface *robot_interface, double loop_rate): inHandController(ns,robot_interface,loop_rate)
{
  bool success=true;
  

  success &= nh_.getParam("force_radius",force_radius_);
  
}

bool tactileController::configureHook()
{
  if(!inHandController::configureHook())
    return false;

  bool success = true;
  // read control parameters from node
  // initialize tactile data:
  string srv_name;
  success &= nh_.getParam("update_srv_name",srv_name);
  update_control_req_=nh_.advertiseService(srv_name,&tactileController::update_params,this);

    // read subscriber topic for tactile force sensing
  string tactile_sub_topic;
  success &= nh_.getParam("tactile_force_topic",tactile_sub_topic);
  

  if(!success)
  {
    return false;
  }

  // create subscriber
  tactile_sub_=nh_.subscribe<biotac_sensors::BioTacForce>(tactile_sub_topic,1,&tactileController::tactile_msg_callback,this);

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
bool tactileController::get_control_cmd(Eigen::VectorXd &q_cmd, Eigen::VectorXd &qd_cmd)
{
  // check for tactile message
  if(!tactile_msg_)
  {
    log("Tactile data not received!");
    return false;
  }
  bool status=tactileControl(q_cmd);

  return status;
  
}

bool tactileController::estimate_friction(const int &finger, Eigen::VectorXd &slip_force)
{
  return true;
}

bool tactileController::update_params(in_hand_controllers::TactileFrictionEstimation::Request &req, in_hand_controllers::TactileFrictionEstimation::Response &res)
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
  Eigen::VectorXd f_slip;
  res.success=estimate_friction(f,f_slip);
  
  q_servo_cmd_=last_jcmd_;
  
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
      tac_contacts[i]=msg.contact.data[i];
    }
    tactile_msg_=true;

  }

  
}

}

