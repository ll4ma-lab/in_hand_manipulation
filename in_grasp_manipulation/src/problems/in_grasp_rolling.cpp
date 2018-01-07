
#define SPARSE_GRADIENT true
#include <in_grasp_manipulation/problems/rolling_problem.hpp>

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
optProblem::optProblem(int time_steps, manipulator_kdl::robotKDL &robot_kdl,vector<double> weights, vector<vector<Eigen::VectorXd>> &des_poses_, vector<double> vel_limits, vector<double> init_joint_angles, vector<int> fingers)
{
  m_dim=robot_kdl.dof*time_steps;
  m_nec=0;//robot_kdl.dof;
  m_nic=robot_kdl.dof*(time_steps);// velocity constraints
  timesteps_=time_steps;
  robot_kdl_=&robot_kdl;
  joints_0=init_joint_angles;
  vel_lim=vel_limits[0];
  wd=weights[0];
  wa=weights[1];
  des_f_poses_=des_poses_;
  fingers_=fingers;
  // given initial object pose and desired object pose:
  

  // get initial contact points
  // generate waypoints for fingers:
  
}

double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;
  Eigen::VectorXd q;
  Eigen::VectorXd obj_pos_err;
  Eigen::VectorXd pos_err,orient_err;
  pos_err.resize(3);
  orient_err.resize(3);
 

  q.resize(robot_kdl_->dof);

  vector<double> o_err;

  double tot_pos_cost=0.0;
  double debug_cost=0.0;
  vector<Eigen::VectorXd> j_poses;
  // store finger pose for min acceleration cost:
  vector<double> init_joints=joints_0;


  Map<Eigen::VectorXd>j_0(&init_joints[0],init_joints.size());

  j_poses.push_back(j_0);
  j_poses.push_back(j_0);
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }
    j_poses.push_back(q);
  }
  // Following Marc's KOMO paper, adding -2,-1, T+1,T+2 poses
  j_poses.push_back(j_poses[timesteps_+1]);
  j_poses.push_back(j_poses[timesteps_+1]);
   
  Eigen::VectorXd acc_cost(16),ee_pose;
  

  Eigen::MatrixXd f_mat,thumb_mat;
  for(int i=0; i<timesteps_;++i)
  {
    double position_cost=0.0;
    double orientation_cost=0.0;

    // Getting joint angles for current timestep:
    q=j_poses[i+2];
    for(int j=0;j<fingers_.size();++j)
    {
      int f=fingers_[f];
      robot_kdl_->getFK(f,q.segment(f*4,4),ee_pose,true);
    
      vector<double> o_1(ee_pose.data()+3, ee_pose.data()+6);
      vector<double> o_d(des_f_poses_[j][i].data()+3, des_f_poses_[j][i].data()+6);
      
      // object pose error:
      
      obj_pos_err=des_f_poses_[j][i]-ee_pose;
      robot_kdl_->euler_diff(o_1,o_d,o_err);
      obj_pos_err[3]=o_err[0]*0.0;
      obj_pos_err[4]=o_err[1]*0.0;
      obj_pos_err[5]=o_err[2]*0.0;

    }   
    // min_acceleration cost:
    acc_cost=(j_poses[i-2+2]-2.0*j_poses[i+2-1]+j_poses[i+2]);
    cost_val+=wa*acc_cost.squaredNorm();

    if(i==timesteps_-1)
    {
      cost_val+=wd*obj_pos_err.squaredNorm();
    }
  }
  // The last timestep is also affected by the extrapolated joint positions:
  int i=timesteps_;
  acc_cost=(j_poses[i-2+2]-2.0*j_poses[i+2-1]+j_poses[i+2]);
  cost_val+=wa*acc_cost.squaredNorm();
  
  return cost_val;
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  
  vector<double> retval(m_dim,0.0);
  Eigen::VectorXd q(robot_kdl_->dof);

  Eigen::VectorXd obj_pose_err;
  Eigen::VectorXd finger_gradient;
  vector<double> gradient;
  gradient.resize(robot_kdl_->dof,0.0);

  obj_pose_err.resize(6);
  

  vector<double> init_joints=joints_0;

  vector<double> o_err;
  Map<Eigen::VectorXd>j_0(&init_joints[0],init_joints.size());
  
  vector<Eigen::VectorXd> j_poses;
  j_poses.push_back(j_0);
  j_poses.push_back(j_0);
  // store finger pose and Jacobian for min acceleration cost:
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }

    j_poses.push_back(q);
  }
  j_poses.push_back(j_poses[timesteps_+1]);
  j_poses.push_back(j_poses[timesteps_+1]);

  Eigen::VectorXd ee_pose;
  Eigen::MatrixXd thumb_mat,f_mat,J;
  Eigen::VectorXd acc_cost(16);
  Eigen::VectorXd grad;

  for(int i=0;i<timesteps_;++i)
  {
    
    q=j_poses[i+2];
    for(int j=0;j<fingers_.size();++j)
    {
      int f=fingers_[j];
      robot_kdl_->getFK(f,q.segment(f*4,4),ee_pose,true);
    
      // compute error using kdl:
      vector<double> o_1(ee_pose.data()+3, ee_pose.data()+6);
      vector<double> o_d(des_f_poses_[j][i].data()+3, des_f_poses_[j][i].data()+6);
      
      robot_kdl_->euler_diff(o_1,o_d,o_err);
      
      obj_pose_err=des_f_poses_[j][i]-ee_pose;

      
      obj_pose_err[3]=o_err[0]*0.0;
      obj_pose_err[4]=o_err[1]*0.0;
      obj_pose_err[5]=o_err[2]*0.0;
      
      if(i==timesteps_-1)
      {
        robot_kdl_->getJacobian(f,q.segment(f*4,4),J);
        finger_gradient=-2.0*wd*obj_pose_err.transpose()*J;
        for (int k=0;k<4;++k) 
        {
          gradient[f*4+k]+=finger_gradient[k];
        }
      }
      
      
      for (int j=0;j<robot_kdl_->dof;++j)
      {
        retval[j+robot_kdl_->dof*(i)]+=gradient[j];
      }
    }
    // min_acceleration gradient:
    acc_cost=wa*(j_poses[i-2+2]-2.0*j_poses[i-1+2]+j_poses[i+2]);
    if(i<timesteps_)
    {
      grad=2.0*acc_cost;
      for (int j=0;j<16;++j)
      {
        retval[j+robot_kdl_->dof*(i)]+=grad[j];
      }
      
    }
    if(i>1)
    {
      grad=2.0*acc_cost;
      for (int j=0;j<16;++j)
      {
        retval[j+robot_kdl_->dof*(i-2)]+=grad[j];
      }
      
    }
    if(i>0)
    {
      grad=-4.0*acc_cost;
      for (int j=0;j<16;++j)
      {
        retval[j+robot_kdl_->dof*(i-1)]+=grad[j];
      }
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

vector<double> optProblem::EqConstraints(vector<double> x) const
{
  // no constraint
  vector<double> ret(m_nec,0.0);
  
  return  ret;
  
}

vector<double> optProblem::eqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nec,0.0);
  // gradient:
  /*
  for(int i=0;i<robot_kdl_.dof;++i)
  {
    if(std::abs(joints_0[i]-x[i])>1e-4)
    {
      if(joints_0[i]-x[i]>0.0)
      {
        retval[i*m_dim+i]=1.0;
      }
      else
      {
        retval[i*m_dim+i]=-1.0;
      }
    }
  }
  */
  return retval;
}
/*
vector<double> optProblem::gradient(const vector<double> x) const
{
  vector<double> grad;
  vector<double> obj_grad=objGradient(x);
  vector<double> eq_grad=eqGradient(x);
  vector<double> ineq_grad=inEqGradient(x);
  
  grad.resize(m_dim+m_dim*m_nec+m_dim*m_nic,0.0);
  for(int i=0;i<m_dim;i++)
  {
    grad[i]=obj_grad[i];

  }

  for(int i=m_dim;i<m_dim+m_dim*m_nec;i++)
  {
    grad[i]=eq_grad[i-m_dim];
  }


  for(int i=m_dim+m_dim*m_nec;i<m_dim+m_dim*m_nec+m_dim*m_nic;i++)
  {
    
    grad[i]=ineq_grad[i-m_dim-m_dim*m_nec];
  }

  return grad;
}
*/
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

  return retval;
}

