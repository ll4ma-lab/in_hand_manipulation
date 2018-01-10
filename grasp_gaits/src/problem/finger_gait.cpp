#include "grasp_gaits/problem/finger_gait.h"

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
optProblem::optProblem(int &time_steps, manipulator_kdl::robotKDL &robot_kdl, Eigen::VectorXd &des_pose, vector<double> &vel_limits, vector<double> &init_joint_angles, int &finger, allegroEnv &c_checker)
{
  m_dim=robot_kdl.dof*time_steps;
  m_nec=0;
  m_nic=robot_kdl.dof*(time_steps)+time_steps;// velocity constraints
  timesteps_=time_steps;
  robot_kdl_=&robot_kdl;
  des_pose_=des_pose;
  joints_0=init_joint_angles;
  vel_lim=vel_limits[0];
  finger_=finger;
  c_check=&c_checker;
}

double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;
  
  Eigen::VectorXd q;
  Eigen::VectorXd pos_err;
 
  q.resize(robot_kdl_->dof);
  
  Eigen::VectorXd ee_pose,obj_pose_err;

  for(int i=0; i<timesteps_;++i)
  {
    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }

    
    robot_kdl_->getFK(finger_,q.segment(finger_*4,4),ee_pose,true);
    
    // object pose error:
    
    obj_pose_err=des_pose_.head(3)-ee_pose.head(3);
    
    
    if(i==timesteps_-1)
    {
      cost_val+=obj_pose_err.squaredNorm();
    }
    else
    {
      //cost_val+=wi*obj_pos_err.squaredNorm()+wr*position_cost+wo*orientation_cost;
    }
  }
 
  return cost_val;
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  vector<double> retval(m_dim,0.0);

  Eigen::VectorXd q;

  Eigen::VectorXd finger_gradient(4);
  finger_gradient.setZero();
  vector<double> gradient;
  gradient.resize(robot_kdl_->dof,0.0);

  q.resize(robot_kdl_->dof);
  
  Eigen::VectorXd ee_pose,obj_pose_err;
  Eigen::MatrixXd J;
  for(int i=0;i<timesteps_;++i)
  {
    
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }
    robot_kdl_->getFK(finger_,q.segment(finger_*4,4),ee_pose,true);
    

      
    obj_pose_err=des_pose_.head(3)-ee_pose.head(3);
    robot_kdl_->getJacobian(finger_,q.segment(finger_*4,4),J);
    if(i==timesteps_-1)
    {
      finger_gradient=-2.0*obj_pose_err.transpose()*J.block(0,0,3,J.cols());
    }
    
    for (int k=0;k<4;++k) 
    {
      gradient[finger_*4+k]+=finger_gradient[k];
    }
   
    for (int j=0;j<robot_kdl_->dof;++j)
    {
      retval[j+robot_kdl_->dof*(i)]+=gradient[j];
    }
    fill(gradient.begin(), gradient.end(), 0.0);
 

  }

  return retval;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  
  // limiting velocity at joints
  vector<double> vel_const(m_nic,0.0);

  
 
  for(int j=0;j<robot_kdl_->dof;++j)
  {
    vel_const[(0)*robot_kdl_->dof+j]=std::abs(x[(0)*robot_kdl_->dof+j]-joints_0[j])-vel_lim;
  }

  for (int i=1; i<timesteps_-1; ++i)
  {
    for(int j=0;j<robot_kdl_->dof;++j)
    {
      vel_const[(i)*robot_kdl_->dof+j]=std::abs(x[(i)*robot_kdl_->dof+j]-x[(i-1)*robot_kdl_->dof+j])-vel_lim;
    }
  }
  //
   // collision const:
  Eigen::VectorXd hand_q(16);
  double collision_err=0.0;

  for(int t=0;t<timesteps_;++t)
  {
    collision_err=0.0;
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      hand_q[j]=x[(t)*robot_kdl_->dof+j];
    }
    
    vector<vector<double>> link_poses;
    link_poses.resize(3*4);
  
    // get link pose for every link:
    Eigen::VectorXd finger_q;
    finger_q.resize(4);
    Eigen::VectorXd l_pose_;
    vector<int> l_idx_={finger_*3,finger_*3+1,finger_*3+2};
        
    for (int j=0;j<4;j++)
    {
      //finger joints:
      finger_q.tail(4)=hand_q.segment(4*j,4);
      for(int i=0;i<3;++i)
      {
        robot_kdl_->getLinkPose(j*3+i,finger_q,l_pose_);
        robot_kdl_->getSTDVector(l_pose_,link_poses[j*3+i]);
      }
    }
    
    // get contact data:
    vector<vector<collision_data>> contact_data;
    c_check->check_contacts(link_poses,contact_data,l_idx_);
    for(int j=0;j<l_idx_.size();++j)
    {
      for(int k=0;k<contact_data[l_idx_[j]].size();++k)
      {
        if(contact_data[l_idx_[j]][k].collision)
        {
          collision_err+=100.0*std::abs(contact_data[l_idx_[j]][k].distance);
        }
        else
        {
          if(-contact_data[l_idx_[j]][k].distance<0.01)
          {
            collision_err+=1.0*std::abs(contact_data[l_idx_[j]][k].distance);
          }
        }
      }
    
    }
    vel_const[robot_kdl_->dof*timesteps_+t]=collision_err;
    
  }

  
  return vel_const;
}


vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(1*robot_kdl_->dof+2*(m_nic-robot_kdl_->dof),0.0);
  for(int j=0;j<robot_kdl_->dof;++j)
  {
    if(x[j]-joints_0[j]<0.0)
    {
      retval[j]=-1.0;
    }
    else
    {
      retval[j]=1.0;
    }
  }
 
  for(int i=1;i<timesteps_;++i)
  {
      for(int j=0;j<robot_kdl_->dof;++j)
      {
        if(x[(i)*robot_kdl_->dof+j]-x[(i-1)*robot_kdl_->dof+j]<0.0)
        {
          retval[1*robot_kdl_->dof+2*(i-1)*robot_kdl_->dof+j]=1.0;
          
          retval[1*robot_kdl_->dof+2*(i-1)*robot_kdl_->dof+robot_kdl_->dof+j]=-1.0;

        }
        else
        {
          retval[1*robot_kdl_->dof+2*(i-1)*robot_kdl_->dof+j]=-1.0;
                                                                
          retval[1*robot_kdl_->dof+2*(i-1)*robot_kdl_->dof+robot_kdl_->dof+j]=1.0;
          
        }
      }
  }
  return retval;

}
vector<double> optProblem::collisionGradient(vector<double> x) const
{
  vector<double> retval(robot_kdl_->dof*timesteps_,0.0);
  Eigen::VectorXd hand_q(16);
  double collision_err=0.0;
  Eigen::VectorXd gradient,finger_gradient;
  finger_gradient.resize(4);
  gradient.resize(16);
  gradient.setZero();
  vector<Eigen::MatrixXd> hand_J(12);
  for(int t=0;t<timesteps_;++t)
  {
    finger_gradient.setZero();
    collision_err=0.0;
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      hand_q[j]=x[(t)*robot_kdl_->dof+j];
    }
    
    vector<vector<double>> link_poses;
    link_poses.resize(3*4);
  
    // get link pose for every link:
    Eigen::VectorXd finger_q;
    finger_q.resize(4);
    Eigen::VectorXd l_pose_;
    vector<int> l_idx_={finger_*3,finger_*3+1,finger_*3+2};

    for (int j=0;j<4;j++)
    {
      //finger joints:
      finger_q.tail(4)=hand_q.segment(4*j,4);
      for(int i=0;i<3;++i)
      {
        robot_kdl_->getLinkJacobian(j*3+i,finger_q,hand_J[j*3+i]);
        robot_kdl_->getLinkPose(j*3+i,finger_q,l_pose_);
        robot_kdl_->getSTDVector(l_pose_,link_poses[j*3+i]);
      }
    }
    
    // get contact data:
    vector<vector<collision_data>> contact_data;
    c_check->check_contacts(link_poses,contact_data,l_idx_);
    for(int j=0;j<4;++j)
    {
      for(int i=0;i<3;++i)
      {
        
        Map<Eigen::VectorXd> link_pose(&link_poses[j*3+i][0],3);
        int n_js=hand_J[j*3+i].cols();
              
        for(int k=0;k<contact_data[j*3+i].size();++k)
        {
          if(contact_data[l_idx_[j]][k].collision)
          {
            Map<Eigen::VectorXd> point(&contact_data[j*3+i][k].point[0],3);
            
            Map<Eigen::VectorXd> dir(&contact_data[j*3+i][k].dir[0],3);
            //cerr<<"pt"<<contact_data[j][k].distance<<endl;
            double p_d=contact_data[j*3+i][k].distance;
            //cerr<<"pd"<<endl;
            Eigen::VectorXd delta_p=-1.0*(link_pose-(point-p_d*dir));
            //cerr<<"Computing gradient"<<endl;
            finger_gradient.segment(0,n_js)+=-100.0*delta_p.transpose()*hand_J[j*3+i].block(0,0,3,n_js);
          }
          else if(-contact_data[l_idx_[j]][k].distance<0.01)
          {
            Map<Eigen::VectorXd> point(&contact_data[j*3+i][k].point[0],3);
            Map<Eigen::VectorXd> dir(&contact_data[j*3+i][k].dir[0],3);
            //cerr<<"pt"<<contact_data[j][k].distance<<endl;
            double p_d=contact_data[j*3+i][k].distance;
            //cerr<<"pd"<<endl;
            Eigen::VectorXd delta_p=-1.0*(link_pose-(point-p_d*dir));
            //cerr<<"Computing gradient"<<endl;
            finger_gradient.segment(0,n_js)+=-1.0*delta_p.transpose()*hand_J[j*3+i].block(0,0,3,n_js);
          }
        }
      }
      gradient.segment(j*4,4)=finger_gradient;
      finger_gradient.setZero();
    }
    
    // add robot collision gradient to main
    for(int k=0;k<robot_kdl_->dof;++k)
    {
      retval[t*robot_kdl_->dof+k]=gradient[k];
    }
  }
  return retval;
}

vector<double> optProblem::EqConstraints(vector<double> x) const
{
  // no constraint
  vector<double> ret(m_nec,0.0);
  
  return  ret;
  
}

vector<double> optProblem::eqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nec,0.0);
  return retval;
}
vector<vector<double>> optProblem::bounds() const
{
  vector<vector<double>> bound;
  vector<double> up_bounds;
  vector<double> low_bounds;
  for(int i=0;i<timesteps_;++i)
  {
    up_bounds.insert(up_bounds.end(), robot_kdl_->up_bounds.begin(), robot_kdl_->up_bounds.end());
    low_bounds.insert(low_bounds.end(), robot_kdl_->low_bounds.begin(), robot_kdl_->low_bounds.end());
  }
  
  bound.resize(2);
  bound[0]=low_bounds;
  bound[1]=up_bounds;
  return bound;
  
}


vector<double> optProblem::sparse_gradient(const vector<double> x) const
{
  vector<double> grad;
  vector<double> obj_grad=objGradient(x);
  //vector<double> eq_grad=eqGradient(x);
  vector<double> ineq_grad=inEqGradient(x);
  vector<double> collision_grad=collisionGradient(x);  
  
  grad.resize(m_dim,0.0);

  for(int i=0;i<m_dim;i++)
  {
    grad[i]=obj_grad[i];
  }

  for(int j=0;j<robot_kdl_->dof;++j)
  {
    grad.emplace_back(ineq_grad[j]);
  }
  
  for(int i=1;i<timesteps_;++i)
  {
    for(int j=0;j<robot_kdl_->dof;++j)
    {
      grad.emplace_back(ineq_grad[1*robot_kdl_->dof+2*(i-1)*robot_kdl_->dof+j]);
      grad.emplace_back(ineq_grad[1*robot_kdl_->dof+2*(i-1)*robot_kdl_->dof+robot_kdl_->dof+j]);
    }
  }
  
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0;j<robot_kdl_->dof;++j)
    {
      grad.emplace_back(collision_grad[i*robot_kdl_->dof+j]);
    }
  }

    // collision constraint:
  return grad;
}
std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> optProblem::gradient_sparsity() const
{
  std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> retval;
  
  // Objective gradient is dense
  for (int i=0;i<m_dim;++i)
  {
    retval.emplace_back(0,i);
  }

  // inequality constraints:
  // Initial joints:
  // inequality constraints:
  for(int i=0;i<robot_kdl_->dof;++i)
  {
    retval.emplace_back(1+m_nec+i,i);
  }
  // Other timesteps:
  //for(int i=0;
  for(int i=1;i<timesteps_;++i)
  {
    for(int j=0;j<robot_kdl_->dof;++j)
    {
      retval.emplace_back(1+m_nec+1*robot_kdl_->dof+(i-1)*robot_kdl_->dof+j, (i-1)*robot_kdl_->dof+j);
      retval.emplace_back(1+m_nec+1*robot_kdl_->dof+(i-1)*robot_kdl_->dof+j,(i)*robot_kdl_->dof+j);
    }
  }
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0;j<robot_kdl_->dof;++j)
    {
      retval.emplace_back(1+m_nec+robot_kdl_->dof*timesteps_+i,i*robot_kdl_->dof+j);
    }
  }


  return retval;
}

