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
  cerr<<srv_name.c_str()<<endl;
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
  surface_sub_=nh_.subscribe<visualization_msgs::MarkerArray>(surface_norm_sub_topic,1,&tactileController::surface_normal_callback,this);
  

  return true;
}

bool tactileController::tactileSlip(const int &finger)
{
  // get force in robot palm frame
  Eigen::MatrixXd T;
  robot_interface_->kdl_->getFK(finger,q_,T);
  if(tac_forces[finger].norm()<0.1)
  {
    return true;
  }
  Eigen::Vector3d palm_f=T.block(0,0,3,3)*tac_forces[finger];
  palm_f.normalize();
  // compare force with surface normal
  if(acos(palm_f.dot(-1.0*surface_normal_[finger]))>atan(mu_slip_))
  {
    return true;
  }
  
  return false;
}
bool tactileController::tactileControl(Eigen::VectorXd &j_cmd)
{
  Eigen::Vector3d imp_center;
  imp_center.setZero();
  vector<Eigen::VectorXd> ftip_poses;
  ftip_poses.resize(f_idx.size());
  for(int i=0;i<f_idx.size();++i)
  {
    robot_interface_->kdl_->getFK(f_idx[i],q_,ftip_poses[i],true);
    imp_center+=ftip_poses[i].head(3);
  }
  // compute impedance centroid
  imp_center=imp_center/f_idx.size();
  j_cmd=last_jcmd_;

  //
  for(int i=0;i<f_idx.size();++i)
  {
    //cerr<<i<<endl;
    // compute force directions
    Eigen::Vector3d f_dir=imp_center-ftip_poses[i];
    f_dir.normalize();
    // project force direction inside friction cone
    // find minimum angle:
    double min_angle= acos(f_dir.dot(-1.0*surface_normal_[f_idx[i]]));
    if(min_angle>atan(mu_))
    {
      // build linear system with three equations
        Eigen::MatrixXd A(3,3);
        A.row(0)=-1.0*surface_normal_[f_idx[i]];
        A.row(1)=(-1.0*surface_normal_[f_idx[i]]).cross(f_dir);
        A.row(2)=f_dir;
        Eigen::VectorXd B(3);
        B[0]=cos(atan(mu_));
        B[1]=0;
        B[2]=cos(min_angle-atan(mu_));
          
        Eigen::VectorXd f_proj = A.lu().solve(B);
        f_dir=f_proj;
    }
    
    
    // compute force magnitude based on slip
    f_mag_[i]=alpha_[i]*f_mag_[i];
    if(tactileSlip(f_idx[i]))
    {
      // increase force magnitude
      f_mag_[i]+=(1-alpha_[i]);
      cerr<<"Slip detected: "<<f_idx[i]<<" "<<f_mag_[i]-0.1<<endl;
    }

    // compute velocity command:
    
    // get joint velocity command:
    // build a kdl chain with an addition link to the current contact point:
    Eigen::MatrixXd J;
    robot_interface_->kdl_->getEEPtJacobian(f_idx[i],q_,tac_cpts[f_idx[i]],J);
    //robot_interface_->kdl_->getJacobian(f_idx[i],q_,J);

    
    Eigen::VectorXd theta_dot=0.01*(f_mag_[i]-0.1)*J.block(0,0,3,J.cols()).transpose()*f_dir;
    j_cmd.segment(f_idx[i]*4,4)+=theta_dot.segment(f_idx[i]*4,4);
    
    if(i==(f_idx.size()-1))
    {
      //robot_interface_->kdl_->getEEPtJacobian(f_idx[f_idx.size()-1],q_,tac_cpts[f_idx[f_idx.size()-1]],J);
      for(int j=0;j<f_idx.size()-1;++j)
      {
        theta_dot=1.0*0.01*(f_mag_[j]-0.1)*J.block(0,0,3,J.cols()).transpose()*f_dir;
        j_cmd.segment(f_idx[f_idx.size()-1]*4,4)+=theta_dot.segment(f_idx[f_idx.size()-1]*4,4);
      
      }
    }
    
  }
  //cerr<<f_mag_.transpose()<<endl;
  //cerr<<j_cmd.transpose()<<endl;
  // send joint velocity as addition to joint position command (minimum force joint position command):
  last_jcmd_=j_cmd;
  
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
  //cerr<<got_surface_normal_<<tactile_msg_<<endl;
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
  f_idx.clear();
  f_idx=vector<int>(std::begin(req.f_idx),std::end(req.f_idx));

  mu_=req.friction_coeff;
  mu_slip_=req.slip_coeff;
  
  // get joint position state that was last commanded
  last_jcmd_.resize(req.last_jcmd.position.size());
  for(int i=0;i<req.last_jcmd.position.size();++i)
  {
    last_jcmd_[i]=req.last_jcmd.position[i];
  }

  res.success=true;
 
  q_servo_cmd_=last_jcmd_;
  f_mag_.resize(f_idx.size());
  alpha_.resize(f_idx.size());
  f_mag_.setIdentity();
  //alpha_.setZero();
  alpha_.setIdentity();
  alpha_=alpha_*0.98;
  f_mag_=f_mag_*0.1/0.98;
  
  des_ftip_poses_.clear();
  des_ftip_poses_.resize(f_idx.size());
  for(int i=0;i<f_idx.size();++i)
  {
    if(req.des_ftip_poses[i].orientation.w==1.0)
    {
      des_ftip_poses_[i]=Eigen::Vector3d(req.des_ftip_poses[i].position.x, req.des_ftip_poses[i].position.y, req.des_ftip_poses[i].position.z);
    }
    else
    {
      des_ftip_poses_[i]=Eigen::Vector3d::Zero();
    }
  }
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
    tac_cpts.resize(msg.contact_pts.size());
    for (int i=0; i<msg.forces.size();++i)
    {
      //int k=f_idx[i];
      tac_forces[i].resize(3);
      tac_forces[i][0]=msg.forces[i].wrench.force.x;
      tac_forces[i][1]=msg.forces[i].wrench.force.y;
      tac_forces[i][2]=msg.forces[i].wrench.force.z;
      tac_cpts[i][0]=msg.contact_pts[i].pose.position.x;
      tac_cpts[i][1]=msg.contact_pts[i].pose.position.y;
      tac_cpts[i][2]=msg.contact_pts[i].pose.position.z;
      //cerr<<tac_cpts[i]<<endl;
      tac_contacts[i]=0;//msg.contact.data[i];
    }
    tactile_msg_=true;
      
  }

  
}
void tactileController::surface_normal_callback(visualization_msgs::MarkerArray msg)
{
  // read surface normal at contact points
  // the surface normal is in robot palm frame, same frame as the fingertip kdl base frame
  surface_normal_.resize(msg.markers.size());
  for (int i=0; i<surface_normal_.size();++i)
  {
    Eigen::Vector3d sn,pt1,pt2;
    pt1[0]=msg.markers[i].points[0].x;
    pt1[1]=msg.markers[i].points[0].y;
    pt1[2]=msg.markers[i].points[0].z;

    pt2[0]=msg.markers[i].points[1].x;
    pt2[1]=msg.markers[i].points[1].y;
    pt2[2]=msg.markers[i].points[1].z;

    surface_normal_[i]=(pt2-pt1).normalized();
    
            
  }
  got_surface_normal_=true;
  
}


}

