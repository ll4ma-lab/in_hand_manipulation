#include "grasp_gaits/problem/ftip_grasp.h"

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
optProblem::optProblem(int &time_steps, manipulator_kdl::robotKDL &robot_kdl, vector<Eigen::VectorXd> &des_pose, vector<double> &vel_limits, vector<double> &init_joint_angles, vector<int> &finger, allegroEnv &c_checker)
{
  m_dim=robot_kdl.dof*time_steps;
  m_nec=0;
  m_nic=robot_kdl.dof*(time_steps)+time_steps+time_steps;// velocity constraints, link collision(t_steps), f_tip contact at T, link collisions, stability constraint
  timesteps_=time_steps;
  robot_kdl_=&robot_kdl;
  des_pose_=des_pose;
  joints_0=init_joint_angles;
  
  vel_lim=vel_limits[0];
  finger_=finger;
  c_check=&c_checker;
  for(int f=0;f<finger_.size();++f)
  {
    //l_idx_.emplace_back(finger_[f]*3,finger_[f]*3+1,finger_[f]*3+2);
  }
  //f_tip_=finger_*3+2;
  // get initial pose:
  Map<Eigen::VectorXd> j0(&joints_0[0],joints_0.size());
  init_joints=j0;
  wa=0.001;
  w_sd=0.0;
}

double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;
  //return cost_val;
  double dist_err=0.0;
  Eigen::VectorXd q;
  Eigen::VectorXd pos_err;
  Eigen::VectorXd acc_cost(robot_kdl_->dof);

  
  q.resize(robot_kdl_->dof);
  
  Eigen::VectorXd ee_pose,obj_pose_err;

  Eigen::VectorXd finger_q;
  finger_q.resize(4);
  Eigen::VectorXd l_pose_;
  vector<vector<double>> link_poses;
  link_poses.resize(3*4);

  vector<Eigen::VectorXd> q_arr;
  Eigen::VectorXd j0(joints_0.size());
  for(int i=0;i<joints_0.size();++i)
  {
    j0[i]=joints_0[i];
  }
  
  q_arr.push_back(j0);
  q_arr.push_back(j0);
  
  for(int i=0; i<timesteps_;++i)
  {
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }
    q_arr.push_back(q);
  }
  q_arr.push_back(q);
  q_arr.push_back(q);

  
  for(int i=0; i<timesteps_;++i)
  {
    // min acc cost:
    acc_cost=q_arr[i-2+2]-2.0*q_arr[i+2-1]+q_arr[i+2];
    cost_val+=wa*acc_cost.squaredNorm();

    
    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }

    if(i==timesteps_-1)
    {
      for(int f=0;f<finger_.size();++f)
      {
        int finger=finger_[f];
        robot_kdl_->getFK(finger,q.segment(finger*4,4),ee_pose,true);
        // object pose error:
        obj_pose_err=des_pose_[f].head(3)-ee_pose.head(3);
        cost_val+=1.0*obj_pose_err.squaredNorm();
        // add cost for contact on object:
           
        vector<double> ftip_pose;
        robot_kdl_->getSTDVector(ee_pose,ftip_pose);
        // get contact data:
        vector<collision_data> contact_data;
        c_check->check_ftip_contact(ftip_pose,contact_data,finger*3+2);
            
        Eigen::VectorXd sd_contact(contact_data.size()); 
        sd_contact.setZero();
        // check for contact
        for(int k=0;k<contact_data.size();++k)
        {
          sd_contact[k]=std::abs(contact_data[k].distance);
        }
        cost_val+=w_sd*(sd_contact.minCoeff());
      }
    }
    else
    {
      // cost to keep finger tip/ links at some threshold distance from object?

      // Update link poses

      // get collision data

      // compute signed distance between object and links

      // ensure they are at certain threshold distance
      
    }
  }
 
  return cost_val;
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  vector<double> retval(m_dim,0.0);
  //return retval;
  Eigen::VectorXd q;
  
  Eigen::VectorXd finger_gradient(4);
  finger_gradient.setZero();
  vector<double> gradient;
  gradient.resize(robot_kdl_->dof,0.0);

  q.resize(robot_kdl_->dof);
  
  Eigen::VectorXd ee_pose,obj_pose_err;
  Eigen::MatrixXd J;
  Eigen::VectorXd acc_cost,acc_grad;

  vector<Eigen::VectorXd> q_arr;
  Eigen::VectorXd j0(joints_0.size());
  for(int i=0;i<joints_0.size();++i)
  {
    j0[i]=joints_0[i];
  }
  
  q_arr.push_back(j0);
  q_arr.push_back(j0);
  
  for(int i=0; i<timesteps_;++i)
  {
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }
    q_arr.push_back(q);
  }
  q_arr.push_back(q);
  q_arr.push_back(q);

  for(int i=0;i<timesteps_;++i)
  {
    acc_cost=wa*(q_arr[i-2+2]-2.0*q_arr[i+2-1]+q_arr[i+2]);
        
    
    acc_grad=2.0*acc_cost;
    for (int j=0;j<robot_kdl_->dof;++j)
    {
      retval[j+robot_kdl_->dof*(i)]+=acc_grad[j];
    }

    if(i>1)
    {
      acc_grad=2.0*acc_cost;
      for (int j=0;j<robot_kdl_->dof;++j)
      {
        retval[j+robot_kdl_->dof*(i-2)]+=acc_grad[j];
      } 
    }
    if(i>0)
    {
      acc_grad=-4.0*acc_cost;
      for (int j=0;j<robot_kdl_->dof;++j)
      {
        retval[j+robot_kdl_->dof*(i-1)]+=acc_grad[j];
      }
     
    }
    
    if(i==timesteps_-1)
    {
      acc_cost=wa*(q_arr[i-1+2]-q_arr[i+2]);
      
      acc_grad=2.0*acc_cost;
      for (int j=0;j<robot_kdl_->dof;++j)
      {
        retval[j+robot_kdl_->dof*(i-1)]+=acc_grad[j];
      }
      acc_grad=-2.0*acc_cost;
      for (int j=0;j<robot_kdl_->dof;++j)
      {
        retval[j+robot_kdl_->dof*(i)]+=acc_grad[j];
      }

    }

    
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }
    if(i==timesteps_-1)
    {
      for(int f=0;f<finger_.size();++f)
      {
        int finger=finger_[f];
        robot_kdl_->getFK(finger,q.segment(finger*4,4),ee_pose,true);
        obj_pose_err=des_pose_[f].head(3)-ee_pose.head(3);
        robot_kdl_->getJacobian(finger,q.segment(finger*4,4),J);
        finger_gradient=-2.0*1.0*obj_pose_err.transpose()*J.block(0,0,3,J.cols());
        // contact:
        vector<double> ftip_pose;
        robot_kdl_->getSTDVector(ee_pose,ftip_pose);
        vector<collision_data> c_data;
        c_check->check_ftip_contact(ftip_pose,c_data,finger*3+2);
      
        Eigen::VectorXd sd_contact(c_data.size());
        sd_contact.setZero();
      
        for(int k=0;k<c_data.size();++k)
        {
          sd_contact[k]=std::abs(c_data[k].distance);
        }
        
        int min_idx;
        sd_contact.minCoeff(&min_idx);
        
        Map<Eigen::VectorXd> point(&c_data[min_idx].point[0],3);
        Map<Eigen::VectorXd> dir(&c_data[min_idx].dir[0],3);
        double p_d=c_data[min_idx].distance;
        Eigen::VectorXd delta_p=-1.0*p_d*dir;

        finger_gradient+=w_sd*delta_p.transpose()*J.block(0,0,3,J.cols());
        for (int k=0;k<4;++k) 
        {
          gradient[finger*4+k]+=finger_gradient[k];
        }
        finger_gradient.setZero();
   
      }
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
  vector<int> f_idx;
  
  // limiting velocity at joints
  vector<double> vel_const(m_nic,0.0);
  Eigen::VectorXd hand_q(16),obj_pose_err;
 
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

  return vel_const;
  /*
  // collision const:
  // Check collision between object and links:
  double collision_err=0.0;
  
  for(int t=0;t<timesteps_;++t)
  {
    //collision_err=0.0;
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

    
    for (int j=0;j<4;j++)
    {
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
    for(int j=0;j<finger_.size();++j)
    {
      int f=finger_[j];
      for (int k=0;k<3;++k)
      {
        // don't check for collision of fingertip at final timestep 
        if(t==timesteps_-1 && k==2)
        {
          Eigen::VectorXd sd_contact(contact_data[l_idx_[f*3+]].size());
          sd_contact.setZero();
          // check for contact
          for(int k=0;k<contact_data[l_idx_[j]].size();++k)
          {
            sd_contact[k]=std::abs(contact_data[l_idx_[j]][k].distance);
          }
          //vel_const[robot_kdl_->dof*timesteps_+timesteps_]=sd_contact.minCoeff();
        }
      
      if(!(t==timesteps_-1 && l_idx_[j]==l_idx_[l_idx_.size()-1]))
      {
        collision_err=0.0;
        for(int k=0;k<contact_data[l_idx_[j]].size();++k)
        {
          if(contact_data[l_idx_[j]][k].collision)
          {
            collision_err+=std::abs(contact_data[l_idx_[j]][k].distance);
          }
        }
      }
    }
    vel_const[robot_kdl_->dof*timesteps_+t]=collision_err;
    return vel_const;
    // check link to link collsions:
    contact_data.clear();

    
    c_check->check_link_collision(contact_data,l_idx_);

    collision_err=0.0;
    for(int j=0; j<l_idx_.size();++j)
    {
      for(int k=0;k<contact_data[l_idx_[j]].size();++k)
      {
        if(contact_data[l_idx_[j]][k].collision)
        {
          collision_err+=std::abs(contact_data[l_idx_[j]][k].distance);
        }
      }
    }
    //vel_const[robot_kdl_->dof*timesteps_+timesteps_+1+t]=collision_err;
  }
  */
  return vel_const;
}


vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(1*robot_kdl_->dof+2*(timesteps_-1)*robot_kdl_->dof,0.0);
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

  vector<double> retval(robot_kdl_->dof*(timesteps_+2+timesteps_),0.0);
  return retval;
  /*
  vector<int> f_idx;
  for(int i=0;i<4;++i)
  {
    if(i!=finger_)
    {
      for(int j=0;j<3;++j)
      {
        f_idx.push_back(i*3+j);
      }
    }
  }
  
  Eigen::VectorXd hand_q(16),obj_pose_err;
  //double collision_err=0.0;
  Eigen::VectorXd gradient,finger_gradient,contact_gradient;
  finger_gradient.resize(4);
  contact_gradient.resize(4);
  gradient.resize(16);
  gradient.setZero();
  vector<Eigen::MatrixXd> hand_J(12);
  for(int t=0;t<timesteps_;++t)
  {
    finger_gradient.setZero();
    gradient.setZero();
    //collision_err=0.0;
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
 
    for (int j=0;j<4;j++)
    {
      if(j==finger_)
      {
        finger_q.tail(4)=hand_q.segment(4*j,4);
      }
      else
      {
        finger_q.tail(4)=init_joints.segment(4*j,4);
      }
      for(int i=0;i<3;++i)
      {
        if(j==finger_)
        {
          robot_kdl_->getLinkJacobian(j*3+i,finger_q,hand_J[j*3+i]);
        }
        robot_kdl_->getLinkPose(j*3+i,finger_q,l_pose_);
        robot_kdl_->getSTDVector(l_pose_,link_poses[j*3+i]);
      }
    }
    
    // get contact data:
    vector<vector<collision_data>> contact_data;
    c_check->check_contacts(link_poses,contact_data,l_idx_);
    
    int j=finger_;

    for(int i=0;i<3;++i)
    {
      int n_js=hand_J[j*3+i].cols();

      if(!(t==timesteps_-1 && i==2))
      {
        
      }
      else
      {
        Eigen::VectorXd sd_contact(contact_data[j*3+i].size());
        sd_contact.setZero();
        // check for contact
        for(int k=0;k<contact_data[j*3+i].size();++k)
        {
          sd_contact[k]=std::abs(contact_data[j*3+i][k].distance);
        }

        int min_idx;
        sd_contact.minCoeff(&min_idx);
        
        Map<Eigen::VectorXd> point(&contact_data[j*3+i][min_idx].point[0],3);
        Map<Eigen::VectorXd> dir(&contact_data[j*3+i][min_idx].dir[0],3);
        double p_d=contact_data[j*3+i][min_idx].distance;
        Eigen::VectorXd delta_p=p_d*dir;
        
        finger_gradient.segment(0,n_js)=1.0*delta_p.transpose()*hand_J[j*3+i].block(0,0,3,n_js);
        for(int k=0;k<4;++k)
        {
          //retval[timesteps_*robot_kdl_->dof+finger_*4+k]=finger_gradient[k];
        }
        finger_gradient.setZero();

      }
      
    }
    
    // add robot collision gradient to main
    for(int k=finger_*4;k<finger_*4+4;++k)
    {
      //retval[t*robot_kdl_->dof+k]+=gradient[k];
    }
        
    gradient.setZero();
    // link collision gradient:
    contact_data.clear();
    c_check->check_link_collision(contact_data,l_idx_);
    for(int i=0;i<3;++i)
    {
      Map<Eigen::VectorXd> link_pose(&link_poses[j*3+i][0],3);
      int n_js=hand_J[j*3+i].cols();

      for(int k=0;k<contact_data[j*3+i].size();++k)
      {
        // 
        if(contact_data[j*3+i][k].collision)
          {
            Map<Eigen::VectorXd> point(&contact_data[j*3+i][k].point[0],3);
            
            Map<Eigen::VectorXd> dir(&contact_data[j*3+i][k].dir[0],3);
            //cerr<<"pt"<<contact_data[j][k].distance<<endl;
            double p_d=contact_data[j*3+i][k].distance;
            Eigen::VectorXd delta_p=p_d*dir;
            //cerr<<"Computing gradient"<<endl;
            finger_gradient.segment(0,n_js)+=delta_p.transpose()*hand_J[j*3+i].block(0,0,3,n_js);
            
          }
      }
     
      gradient.segment(j*4,4)+=finger_gradient;
      finger_gradient.setZero();
     }
    //cerr<<gradient.transpose()<<endl;
    for(int k=finger_*4;k<finger_*4+4;++k)
    {
      //retval[timesteps_*robot_kdl_->dof+robot_kdl_->dof+t*robot_kdl_->dof+k]=gradient[k];
    }
    gradient.setZero();
  }
  return retval;
  */
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

  // collision gradient only affects the finger:
  // TODO: change index to affect only finger_ joints.
  for(int i=0;i<timesteps_;++i)
  {
    
    for(int j=0;j<robot_kdl_->dof;++j)
    {
      //grad.emplace_back(collision_grad[i*robot_kdl_->dof+j]);
    }
  }

  

  // link collision gradient:
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0;j<robot_kdl_->dof;++j)

    {
      //grad.emplace_back(collision_grad[timesteps_*robot_kdl_->dof+robot_kdl_->dof+i*robot_kdl_->dof+j]);
    }
  }

  

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

  // object collision gradient
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0;j<robot_kdl_->dof;++j)

    {
      //retval.emplace_back(1+m_nec+robot_kdl_->dof*timesteps_+i,i*robot_kdl_->dof+j);
    }
  }
  // link collision gradient
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0;j<robot_kdl_->dof;++j)
    {
      //retval.emplace_back(1+m_nec+robot_kdl_->dof*timesteps_+timesteps_+1+i,i*robot_kdl_->dof+j);
    }
  }

  return retval;
}
