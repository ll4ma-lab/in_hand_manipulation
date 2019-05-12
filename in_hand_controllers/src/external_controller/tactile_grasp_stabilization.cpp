#include <in_hand_controllers/external_controller/tactile_grasp_stabilization.h>
namespace hand_task_controller
{

tactileGraspStabilization::tactileGraspStabilization(string ns, robot_interface::RobotInterface *robot_interface, double loop_rate): inHandController(ns,robot_interface,loop_rate)
{
  bool success=true;
  // read eigen grasp parameters

  vector<int> eigen_idx;
  vector<double> eigen_amplitude,eigen_preshape;
  string eigen_ns;
  success &= nh_.getParam("eigen_ns",eigen_ns);
  
  success &= nh_.getParam(eigen_ns+"eigen_grasps",eigen_idx);
  
  success &= nh_.getParam(eigen_ns+"eigen_amplitude",eigen_amplitude);
  success &= nh_.getParam(eigen_ns+"eigen_preshape",eigen_preshape);

  success &= nh_.getParam("grasp_control_params/stiffness_delta",stiffness_delta_);
  success &= nh_.getParam("grasp_control_params/f_deviation",deviation_force_);
  vector<vector<double>> eigen_grasp;
  eigen_grasp.resize(eigen_idx.size());
  for(int i=0;i<eigen_idx.size();++i)
  {
    ostringstream os;
    
    os<<"eigen_grasp_"<<eigen_idx[i];
    string e_name=os.str();
    
    success &= nh_.getParam(e_name,eigen_grasp[i]);
    
  }

  
  eigen_grasp_=grasping::eigenGrasp(eigen_grasp,eigen_preshape,eigen_amplitude);

}

bool tactileGraspStabilization::configureHook()
{
  if(!inHandController::configureHook())
    return false;

  bool success = true;
  // read control parameters from node
  // initialize tactile data:
  string srv_name;
  success &= nh_.getParam("update_srv_name",srv_name);
  update_control_req_=nh_.advertiseService(srv_name,&tactileGraspStabilization::update_params,this);

    // read subscriber topic for tactile force sensing
  string tactile_sub_topic;
  success &= nh_.getParam("tactile_force_topic",tactile_sub_topic);
  

  if(!success)
  {
    return false;
  }

  // create subscriber
  tactile_sub_=nh_.subscribe<biotac_sensors::BioTacForce>(tactile_sub_topic,1,&tactileGraspStabilization::tactile_msg_callback,this);

  return true;
}

bool tactileGraspStabilization::tactileSlip()
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
bool tactileGraspStabilization::tactileControl(Eigen::VectorXd &j_cmd)
{
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
  return false;

}
bool tactileGraspStabilization::get_control_cmd(Eigen::VectorXd &q_cmd, Eigen::VectorXd &qd_cmd)
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
    
bool tactileGraspStabilization::update_params(in_hand_controllers::TactileGraspControl::Request &req, in_hand_controllers::TactileGraspControl::Response &res)
{
  // get number of fingers
  f_idx.resize(req.f_idx.size());
  for(int i=0;i<req.f_idx.size();++i)
  {
    f_idx[i]=req.f_idx[i];
  }
  grasp_ampl_.resize(eigen_grasp_.eg_size);
  // get eigen amplitudes
  for (int i=0;i<req.eigen_ampl.size();++i)
  {
    grasp_ampl_[i]=req.eigen_ampl[i];
  }

  
  // get joint position state that was last commanded
  last_jcmd_.resize(req.last_jcmd.position.size());
  for(int i=0;i<req.last_jcmd.position.size();++i)
  {
    last_jcmd_[i]=req.last_jcmd.position[i];
  }
  q_servo_cmd_=last_jcmd_;
  cerr<<q_servo_cmd_.transpose()<<endl;
  res.success=true;
  log("Parameters updated!");
  return true;
                   
  
}

void tactileGraspStabilization::tactile_msg_callback(biotac_sensors::BioTacForce msg)
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

