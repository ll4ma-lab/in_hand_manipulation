/*
 * Copyright (C) 2018  Balakumar Sundaralingam, LL4MA Lab, University of Utah
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "in_hand_controllers/hand_services/eigen_control_interface.h"

namespace hand_services
{
namespace control_interface
{
bool eigenControlInterface::configureHook()
{
  bool success=controlInterface::configureHook();
  // load eigen metrics
  eigen_space_=false;
  success &= nh_.getParam("eigen_space",eigen_space_);
  if(!success)
  {
    log("eigen_space flag missing in config file!");
    return false;
  }
  if(eigen_space_)
  {
    vector<int> eigen_idx;
    vector<double> eigen_amplitude,eigen_preshape;
  
    success &= nh_.getParam("eigen_grasps",eigen_idx);
  
    success &= nh_.getParam("eigen_amplitude",eigen_amplitude);
    success &= nh_.getParam("eigen_preshape",eigen_preshape);

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
  
  if(!success)
  {
    log("Failed to load eigen params from param server.", ERROR);
    return false;
  }

  // re-initialize service request:
  control_req_.shutdown();
  control_req_=nh_.advertiseService(hand_cmd_srv_,&eigenControlInterface::handCommandEigen,this);

  log("Eigen initialization complete.");
  return true;  
}

bool eigenControlInterface::handCommandEigen(hand_services::SendHandCommand::Request &req, hand_services::SendHandCommand::Response &resp)
{
  log("Eigen hand command received!");
  // hand command processing function:
  vector<int> fingers(req.fingers.size(),0);
  for(int i=0;i<fingers.size();++i)
    fingers[i]=req.fingers[i];
  
  if(req.control_mode==0)
  {
    //cerr<<num_jnts_<<endl;
    log("Runnning Joint position control mode");
    if(req.des_js.position.size()==num_jnts_)
    {
    Eigen::Map<Eigen::VectorXd> q_des(&req.des_js.position[0],req.des_js.position.size());
    resp.reached=sendDesJPos(q_des);
    }
    else if(eigen_space_)
    {
      vector<double> eigen_amplitudes(eigen_grasp_.eg_size,0.0);
      for(int i=0;i<eigen_grasp_.eg_size;++i)
      {
        eigen_amplitudes[i]=req.data[i];
      }
      // get joint cmd from eigen space:
      vector<double> des_js=eigen_grasp_.get_state(eigen_amplitudes);
      Eigen::Map<Eigen::VectorXd> q_des(&des_js[0],des_js.size());
      
      resp.reached=sendDesJPos(q_des);
    }
    else
    {
      log("Desired joint position command has incorrect number of joints!!",ERROR);
    }
    resp.last_des_cmd=req.des_js;
  }
  if(req.control_mode==1)
  {
         
    log("Runnning Joint position trajectory control mode");
    if(tactile_enabled_ && req.biotac)
    {
      log("tactile sensing active");
      
      resp.reached=sendTrajectory(req.des_jtraj,true,fingers);
    }
    else
    {
      if(req.biotac)
        log("Tactile sensor not initialized, running trajectory without contact sensing",WARN);
      
      resp.reached=sendTrajectory(req.des_jtraj,false,fingers);
    }
  }
  if(req.control_mode==2)
  {
    log("Grasping object");    
    vector<int> f_idx={0};// TODO: add fingertip contact detection
    vector<int> j_idx(req.joints.size(),0);
    for(int i=0;i<req.joints.size();++i)
      j_idx[i]=req.joints[i];

    Eigen::VectorXd q_current_cmd;
    graspControl(f_idx,j_idx,q_current_cmd);
    sensor_msgs::JointState last_cmd;
    last_cmd.position.resize(q_current_cmd.size());
    for(int i=0;i<q_current_cmd.size();++i)
    {
      last_cmd.position[i]=q_current_cmd[i];
    }
    resp.last_des_cmd=last_cmd;
  }

  if(req.control_mode==3)
  {
    vector<int> f_idx(req.fingers.size());
    for(int i=0;i<f_idx.size();++i)
    {
      f_idx[i]=req.fingers[i];
    }
    if(eigen_space_==false)
    {
      log("Jacobian Contact Controller");
      // Move to initial joint angles:
      //Map<Eigen::VectorXd> q_des(&req.des_js.position[0],req.des_js.position.size());
      //resp.reached=sendDesJPos(q_des);
      //log("moved to initial joint config");
      // get approach vector from serialized data:
      vector<Eigen::Vector3d> app_vectors(req.fingers.size());
      for(int i=0;i<req.fingers.size();++i)
      {
        app_vectors[i][0]=req.data[i*3];
        app_vectors[i][1]=req.data[i*3+1];
        app_vectors[i][2]=req.data[i*3+2];
      }
      Eigen::VectorXd q_current_cmd;

      // Run Jacobian controller until contact is detected:
      bool contact=approachContact(req.biotac,fingers,app_vectors,q_current_cmd);
      sensor_msgs::JointState last_cmd;
      last_cmd.position.resize(q_current_cmd.size());
      for(int i=0;i<q_current_cmd.size();++i)
      {
        last_cmd.position[i]=q_current_cmd[i];
      }
      resp.last_des_cmd=last_cmd;
    }
    else
    {
      log("Eigen Contact Controller");
      vector<double> eigen_amplitudes(eigen_grasp_.eg_size,0.0);
      for(int i=0;i<eigen_grasp_.eg_size;++i)
      {
        eigen_amplitudes[i]=req.data[i];
      }
      Eigen::VectorXd q_current_cmd;
      bool contact= approachContactEigen(req.biotac,fingers,eigen_amplitudes,q_current_cmd);
      sensor_msgs::JointState last_cmd;
      last_cmd.position.resize(q_current_cmd.size());
      for(int i=0;i<q_current_cmd.size();++i)
      {
        last_cmd.position[i]=q_current_cmd[i];
      }
      resp.last_des_cmd=last_cmd;

    }
    // Run force threshold controller
    
  }

  if(req.control_mode==4)
  {
    vector<int> f_idx(req.fingers.size());
    for(int i=0;i<f_idx.size();++i)
    {
      f_idx[i]=req.fingers[i];
    }

    log("increasing stiffness");
    Eigen::VectorXd j_cmd;
    Eigen::Map<Eigen::VectorXd> q_des(&req.des_js.position[0],req.des_js.position.size());
    
    if(eigen_space_==false)
    {
       j_cmd=q_des;
   
      vector<int> j_idx(req.joints.size(),0);
      for(int i=0;i<req.joints.size();++i)
        j_idx[i]=req.joints[i];
      
      resp.reached=increaseStiffness(j_idx,q_des,j_cmd);
     
    }
    else
    {
      vector<double> eigen_amplitudes(eigen_grasp_.eg_size,0.0);
      for(int i=0;i<eigen_grasp_.eg_size;++i)
      {
        eigen_amplitudes[i]=req.data[i];
      }
      Eigen::VectorXd q_current_cmd;
      
      resp.reached=increaseStiffnessEigen(f_idx,q_des,eigen_amplitudes,j_cmd);
      
    }
    sensor_msgs::JointState last_cmd;
    last_cmd.position.resize(j_cmd.size());
    for(int i=0;i<j_cmd.size();++i)
    {
      last_cmd.position[i]=j_cmd[i];
    }
    resp.last_des_cmd=last_cmd;
  }

  if(req.control_mode==5)
  {
    log("Connecting to external controller");
    // subscribe to external controller command topic:
    if(!req.string_data.empty())
    {

      log("External controller not supported yet..");
      //external_cmd_sub_=nh_.subscribe<sensor_msgs::JointState>(req.string_data[0], 1, &eigenControlInterface::externalCmdCallback, this);
      external_control_=true;
    }
    else
    {
      log("External joint cmd topic not specified!!!");
    }

  }
  if(req.control_mode==6)
  {
    log("Disconnecting from external controller");
    external_cmd_sub_.shutdown();
    external_control_=false;
  }

  if(req.control_mode==7)
  {
    log("resetting biotac tare");
    biotac_sensors::UpdateTare tare_msg;
    tare_msg.request.fingers=req.fingers;
    if(tare_reset_srv_.call(tare_msg))
    {
      resp.reached=tare_msg.response.updated;
    }
    else
    {
      log("Failed to call tare reset service",ERROR);
    }
  }

  if(req.control_mode==8)
  {
    log("Reaching desired force on fingertips by Jacobian servoing");
    vector<Eigen::Vector3d> app_vectors(req.fingers.size());
    vector<double> force_mag(req.fingers.size());

    for(int i=0;i<req.fingers.size();++i)
    {
      app_vectors[i][0]=req.data[i*3];
      app_vectors[i][1]=req.data[i*3+1];
      app_vectors[i][2]=req.data[i*3+2];
      force_mag[i]=req.data[req.fingers.size()*3+i];
    }

    bool force_reached=forceServoing(fingers,force_mag,app_vectors);
  }
  
  if(req.control_mode==-1)
  {
    log("Immediate joint position command is currently disabled");
    resp.reached=false;
  }
  return true;
}
bool eigenControlInterface::approachContactEigen(const  bool &tactile_sensing, const vector<int> &fingers, const vector<double> &eigen_amplitudes, Eigen::VectorXd &last_pos_cmd)
{
  // get eigen space delta
  Eigen::MatrixXd J;
  eigen_grasp_.get_jacobian(J);
  Eigen::VectorXd theta_dot;
  Eigen::VectorXd alpha_dot;
  alpha_dot.resize(eigen_grasp_.eg_size);

  Eigen::VectorXd q_dot_cmd(16);
  q_dot_cmd.setZero();

  for(int i=0;i<eigen_grasp_.eg_size;++i)
  {
    alpha_dot[i]=eigen_amplitudes[i]-eigen_grasp_.a_range_[0];
  }
  theta_dot=J*alpha_dot;
  theta_dot=theta_dot/300.0;
  Eigen::VectorXd q_current_cmd=q_;

  // Velocity control does not work well
  //double min_vel=0.04; 
  robot_interface::ControlMode cm=robot_interface::ControlMode::POS;// 0==position
  Eigen::VectorXd fingers_contact(fingers.size());
  fingers_contact.setZero();
  while(fingers_contact.sum()<fingers.size())
  {

    // get current joint state:
    ros::spinOnce();
    for(int i=0;i<fingers.size();++i)
    {
      if(fingers_contact[i]==0)
      {
        bool contact_reached=checkContact(fingers[i]);
        if(contact_reached)
        {
          fingers_contact[i]=1;
          cerr<<"Contact detected on Finger "<<fingers[i]<<endl;
          q_dot_cmd.segment(4*fingers[i],4).setZero();
        }
        else
        {
          // threshold approach velocity by scaling
          Eigen::VectorXd q_dot_f=theta_dot.segment(4*fingers[i],4);
          q_dot_f=approach_vel_*q_dot_f/q_dot_f.lpNorm<Eigen::Infinity>();
          
          q_dot_cmd.segment(4*fingers[i],4)=q_dot_f;
        }
      }
      
    }
    q_current_cmd+=q_dot_cmd;
    robot_interface_->publishRealRobot(q_current_cmd,cm);

    rate_.sleep();
  }
  // send current state:
  Eigen::VectorXd q_des=q_current_cmd;
  cm=robot_interface::ControlMode::POS;
  robot_interface_->publishRealRobot(q_des,cm);

  last_pos_cmd=q_current_cmd;
  
  return true;
}
bool eigenControlInterface::increaseStiffnessEigen(const vector<int> &fingers,const Eigen::VectorXd &q_des, const vector<double> &eigen_amplitudes, Eigen::VectorXd &last_pos_cmd)
{
  // get eigen space delta
  Eigen::MatrixXd J;
  eigen_grasp_.get_jacobian(J);
  Eigen::VectorXd theta_dot;
  Eigen::VectorXd alpha_dot;
  alpha_dot.resize(eigen_grasp_.eg_size);

  Eigen::VectorXd q_dot_cmd(16);
  q_dot_cmd.setZero();

  for(int i=0;i<eigen_grasp_.eg_size;++i)
  {
    alpha_dot[i]=eigen_amplitudes[i]-eigen_grasp_.a_range_[0];
  }
  theta_dot=J*alpha_dot;
  theta_dot=theta_dot/300.0;
  Eigen::VectorXd q_current_cmd=q_des;

  // Velocity control does not work well
  //double min_vel=0.04; 
  robot_interface::ControlMode cm=robot_interface::ControlMode::POS;// 0==position
  Eigen::VectorXd fingers_contact(fingers.size());
  fingers_contact.setZero();
  
  // get current joint state:
  //ros::spinOnce();
  for(int i=0;i<fingers.size();++i)
  {

    // threshold approach velocity by scaling
    Eigen::VectorXd q_dot_f=theta_dot.segment(4*fingers[i],4);
    q_dot_f=stiffness_delta_*q_dot_f/q_dot_f.lpNorm<Eigen::Infinity>();
    if(i<3)
    {
      q_dot_f=0.3*q_dot_f;
    }
    q_dot_cmd.segment(4*fingers[i],4)=q_dot_f;
    
    
  }
  for(int i=0;i<100;++i)
  {
    q_current_cmd+=q_dot_cmd;
    robot_interface_->publishRealRobot(q_current_cmd,cm);
    rate_.sleep();

  }

  // send current state:

  last_pos_cmd=q_current_cmd;
  
  return true;
}

}
}
