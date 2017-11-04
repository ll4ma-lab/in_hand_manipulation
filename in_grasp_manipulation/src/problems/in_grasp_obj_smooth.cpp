
#define SPARSE_GRADIENT true
#include <ll4ma_planner/problems/opt_problem.hpp>

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
/*
optProblem::optProblem(int time_steps, manipulator_kdl::robotKDL robot_kdl, vector<vector<double>> data_arr, vector<double> vel_limxits, vector<double> init_joint_angles)
{
  m_dim=robot_kdl.dof*time_steps;
  m_nec=0;//robot_kdl.dof;
  m_nic=robot_kdl.dof*(time_steps);// velocity constraints
  timesteps_=time_steps;
  robot_kdl_=robot_kdl;
  des_poses.resize(time_steps);
  joints_0=init_joint_angles;
  vel_lim=vel_limits[0];
  data_arr_=data_arr;

  Map<Eigen::VectorXd> des_pose(&data_arr[0][0],data_arr[0].size());// desired object pose
  Map<Eigen::VectorXd> initial_pose(&data_arr[4+data_arr[3].size()][0],data_arr[4+data_arr[3].size()].size());// desired object pose
  //des_poses[0]=initial_pose;
  
  //cerr<<"Initial pose: "<<endl;
  //cerr<<des_poses[0].transpose()<<endl;
  for(int i=0;i<time_steps;++i)
  {
    des_poses[i]=initial_pose+(des_pose-initial_pose)*(float(i+1)/float(time_steps-1));
    //cerr<<des_poses[i].transpose()<<endl;
  }
  //cerr<<"final pose:"<<endl;
  //cerr<<des_pose.transpose()<<endl;
}
*/
// Initializer
optProblem::optProblem(int time_steps, manipulator_kdl::robotKDL robot_kdl, vector<vector<double>> data_arr,vector<Eigen::MatrixXd> data_mats, vector<double> vel_limits, vector<double> init_joint_angles)
{
  m_dim=robot_kdl.dof*time_steps;
  m_nec=0;//robot_kdl.dof;
  m_nic=robot_kdl.dof*(time_steps);// velocity constraints
  timesteps_=time_steps;
  robot_kdl_=robot_kdl;
  des_poses.resize(time_steps);
  joints_0=init_joint_angles;
  vel_lim=vel_limits[0];
  data_arr_=data_arr;
  data_mats_=data_mats;
  Map<Eigen::VectorXd> des_pose(&data_arr[0][0],data_arr[0].size());// desired object pose
  Map<Eigen::VectorXd> initial_pose(&data_arr[4+data_arr[3].size()][0],data_arr[4+data_arr[3].size()].size());// desired object pose
  for(int i=0;i<time_steps;++i)
  {
    des_poses[i]=initial_pose+(des_pose-initial_pose)*(float(i+1)/float(time_steps));
    //cerr<<des_poses[i].transpose()<<endl;
  }
  //cerr<<"final pose:"<<endl;
  //cerr<<des_pose.transpose()<<endl;
}

double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;
  vector<double> q;
  vector<double> ee_pose_arr;
  Eigen::VectorXd obj_pos_err;
  Eigen::VectorXd pos_err,orient_err;
  pos_err.resize(3);
  orient_err.resize(3);
  int rigid_f=int(data_arr_[2][0]);// rigid_finger

  // position offsets:
  vector<Eigen::VectorXd> pos_offsets;
  
  for(int k=0;k<data_arr_[3].size();++k)
  {
    vector<double> p_off=data_arr_[4+k];
    Map<Eigen::VectorXd> pos_offset(&p_off[0],p_off.size());
    pos_offsets.emplace_back(pos_offset);
  }
  double wo,wr,wd,wi;
  wo=data_arr_[1][0];
  wr=data_arr_[1][1];
  wd=data_arr_[1][2];
  wi=data_arr_[1][3];
  q.resize(robot_kdl_.dof,0.0);

  vector<double> o_err;

  double tot_pos_cost=0.0;
  double debug_cost=0.0;
  vector<Eigen::VectorXd> r_f_poses;
  // store finger pose for min acceleration cost:
  vector<double> init_joints=joints_0;
  vector<double> r_j0(&init_joints[rigid_f*4],&init_joints[rigid_f*4+4]);
  vector<double> pose_0_=robot_kdl_.getFK(rigid_f,r_j0,true);
  Map<Eigen::VectorXd>pose_0(&pose_0_[0],6);

  r_f_poses.push_back(pose_0);
  r_f_poses.push_back(pose_0);

  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }
    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);
    ee_pose_arr=robot_kdl_.getFK(rigid_f,f_q,true);
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);
    r_f_poses.push_back(ee_pose);
  }
  // Following Marc's KOMO paper, adding -2,-1, T+1,T+2 poses
  //cerr<<r_f_poses.size()<<endl;
  r_f_poses.push_back(r_f_poses[timesteps_+1]);
  r_f_poses.push_back(r_f_poses[timesteps_+1]);
   
  Eigen::VectorXd acc_cost(6);
  

  for(int i=0; i<timesteps_;++i)
  {
    double position_cost=0.0;
    double orientation_cost=0.0;

    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }
    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);
    
    ee_pose_arr=robot_kdl_.getFK(rigid_f,f_q,true);
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);
    vector<double> o_1(ee_pose.data()+3, ee_pose.data()+6);
    vector<double> o_d(des_poses[i].data()+3, des_poses[i].data()+6);
    
    // object pose error:
    
    obj_pos_err=des_poses[i]-ee_pose;
    
    o_err=robot_kdl_.euler_diff(o_1,o_d);
    obj_pos_err[3]=o_err[0]*0.1;
    obj_pos_err[4]=o_err[1]*0.1;
    obj_pos_err[5]=o_err[2]*0.1;

    // Relaxed-Rigidity costs:
    // get current thumb mat:
    Eigen::MatrixXd  thumb_mat=robot_kdl_.getFKEigen(rigid_f,f_q);
    
    for(int k=0;k<data_arr_[3].size();++k)
    {
      int f=data_arr_[3][k];
      if(f!=rigid_f)
      {
        // Position:
        vector<double> f_joints(&q[f*4],&q[f*4+4]);

        Eigen::MatrixXd f_mat=robot_kdl_.getFKEigen(f,f_joints);
        Eigen::MatrixXd des_mat=thumb_mat*data_mats_[f];


	// compute difference between the two frames:
        
	
        Eigen::VectorXd pose_diff(6);
        
        pose_diff[0]=des_mat(0,3)-f_mat(0,3);
        pose_diff[1]=des_mat(1,3)-f_mat(1,3);
        pose_diff[2]=des_mat(2,3)-f_mat(2,3);
        
        
        // Compute orientation error:
        o_err=robot_kdl_.euler_diff(f_mat,des_mat);
        
        pos_err=pose_diff.segment(0,3);
        
        orient_err[1]=o_err[1];
        orient_err[0]=0.0;
        orient_err[2]=0.0;
        position_cost+=pos_err.squaredNorm();
        orientation_cost+=orient_err.squaredNorm();
      }
    }
   
    tot_pos_cost+=position_cost;
   
    // min_acceleration cost:
    //if(i>1)
    {
      acc_cost=wi*(r_f_poses[i-2+2]-2.0*r_f_poses[i-1+2]+r_f_poses[i+2]);
      acc_cost[3]=0.0;
      acc_cost[4]=0.0;
      acc_cost[5]=0.0;
      cost_val+=acc_cost.squaredNorm();
      
      acc_cost=wi*(r_f_poses[i+2+2]-2.0*r_f_poses[i+1+2]+r_f_poses[i+2]);
      acc_cost[3]=0.0;
      acc_cost[4]=0.0;
      acc_cost[5]=0.0;
      cost_val+=acc_cost.squaredNorm();
      //debug_cost+=wi*acc_cost.squaredNorm();
      //cerr<<cost_val<<endl;
    }
    
    cost_val+=wr*position_cost+wo*orientation_cost;
  }
  //cerr<<tot_pos_cost<<endl;
  //cerr<<debug_cost<<endl;
  return cost_val;
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  double wo,wr,wd,wi;
  wo=data_arr_[1][0];
  wr=data_arr_[1][1];
  wd=data_arr_[1][2];
  wi=data_arr_[1][3];
  
  vector<double> retval(m_dim,0.0);
  vector<double> q;
  vector<double> ee_pose_arr;

  Eigen::VectorXd obj_pose_err;
  Eigen::VectorXd finger_gradient;
  vector<double> gradient;
  gradient.resize(robot_kdl_.dof,0.0);

  obj_pose_err.resize(6);
  q.resize(robot_kdl_.dof,0.0);
  int rigid_f=int(data_arr_[2][0]);// rigid_finger
  

  Eigen::VectorXd pos_err,orient_err;
  pos_err.resize(6);
  orient_err.resize(6);
  vector<double> init_joints=joints_0;
  vector<double> o_err;
  vector<double> r_j0(&init_joints[rigid_f*4],&init_joints[rigid_f*4+4]);
  vector<double> pose_0_=robot_kdl_.getFK(rigid_f,r_j0,true);
  Map<Eigen::VectorXd>pose_0(&pose_0_[0],6);
  // position offsets:
  vector<Eigen::VectorXd> pos_offsets;
  
  for(int k=0;k<data_arr_[3].size();++k)
  {
    vector<double> p_off=data_arr_[4+k];
    Map<Eigen::VectorXd> pos_offset(&p_off[0],p_off.size());
    pos_offsets.emplace_back(pos_offset);
  }

  vector<Eigen::MatrixXd> r_f_Jacobian;
  vector<Eigen::VectorXd> r_f_poses;
  r_f_poses.push_back(pose_0);
  r_f_poses.push_back(pose_0);

  // store finger pose and Jacobian for min acceleration cost:
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }
    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);
    ee_pose_arr=robot_kdl_.getFK(rigid_f,f_q,true);
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);
    r_f_poses.push_back(ee_pose);
    r_f_Jacobian.push_back(robot_kdl_.getJacobian(rigid_f,f_q));
  }
  
  r_f_poses.push_back(r_f_poses[timesteps_+1]);
  r_f_poses.push_back(r_f_poses[timesteps_+1]);
   
  for(int i=0;i<timesteps_;++i)
  {
    
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }
    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);

    
    ee_pose_arr=robot_kdl_.getFK(rigid_f,f_q,true);
    
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);// only position

    // compute error using kdl:
    vector<double> o_1(ee_pose.data()+3, ee_pose.data()+6);
    vector<double> o_d(des_poses[i].data()+3, des_poses[i].data()+6);
    
    o_err=robot_kdl_.euler_diff(o_1,o_d);
      
    obj_pose_err=des_poses[i]-ee_pose;

    
    obj_pose_err[3]=o_err[0]*0.1;
    obj_pose_err[4]=o_err[1]*0.1;
    obj_pose_err[5]=o_err[2]*0.1;

    if(i==timesteps_-1)
    {
      finger_gradient=-2.0*wd*obj_pose_err.transpose()*robot_kdl_.getJacobian(rigid_f,f_q);//r_f_Jacobian[i];
      for (int k=0;k<4;++k) 
      {
        gradient[rigid_f*4+k]+=finger_gradient[k];
      }
    }
    
   

     // Relaxed-Rigidity costs:
    Eigen::MatrixXd  thumb_mat=robot_kdl_.getFKEigen(rigid_f,f_q);
    Eigen::MatrixXd inv_thumb_mat=thumb_mat.inverse();

    for(int k=0;k<data_arr_[3].size();++k)
    {
      int f=data_arr_[3][k];
      if(f!=rigid_f)
      {
        // Position:
        vector<double> f_joints(&q[f*4],&q[f*4+4]);
        
        Eigen::MatrixXd f_mat=robot_kdl_.getFKEigen(f,f_joints);

        Eigen::MatrixXd des_mat=thumb_mat*data_mats_[f];


	// compute difference between these two frames:
	// off_mat and data_mat
        Eigen::VectorXd pose_diff(6);
        pose_diff[0]=des_mat(0,3)-f_mat(0,3);
        pose_diff[1]=des_mat(1,3)-f_mat(1,3);
        pose_diff[2]=des_mat(2,3)-f_mat(2,3);

        o_err=robot_kdl_.euler_diff(f_mat,des_mat);
        
        pos_err=pose_diff;//.segment(0,3);
        //orient_err=pose_diff;//.segment(3,3);
        pos_err[0]=wr*pos_err[0];
        pos_err[1]=wr*pos_err[1];
        pos_err[2]=wr*pos_err[2];
        pos_err[3]=0.0;
        pos_err[4]=0.0;
        pos_err[5]=0.0;
        
        pos_err[4]=wo*(o_err[1]);
        
        finger_gradient=-2.0*pos_err.transpose()*robot_kdl_.getJacobian(f,f_joints);
        for (int l=0;l<4;++l)
        {
          gradient[f*4+l]+=finger_gradient[l];
        }

      }
    }

    
    // min_acceleration gradient:
    {
      Eigen::VectorXd acc_cost(6);
      acc_cost.setZero();
      acc_cost=wi*(r_f_poses[i-2+2]-2.0*r_f_poses[i-1+2]+r_f_poses[i+2]);
      
      acc_cost[3]=0.0;
      acc_cost[4]=0.0;
      acc_cost[5]=0.0;
      
      if(i>1)
      {
	finger_gradient=2.0*acc_cost.transpose()*r_f_Jacobian[i-2];
	for (int j=rigid_f*4;j<rigid_f*4+4;++j)
	{
	  retval[j+robot_kdl_.dof*(i-2)]+=finger_gradient[j-rigid_f*4];
	}
      }
      if(i>0)
      {
        finger_gradient=-4.0*acc_cost.transpose()*r_f_Jacobian[i-1];
        for (int j=rigid_f*4;j<rigid_f*4+4;++j)
	{
	  retval[j+robot_kdl_.dof*(i-1)]+=finger_gradient[j-rigid_f*4];
	}

      }
      if(i<timesteps_-1)
      {
        finger_gradient=2.0*acc_cost.transpose()*r_f_Jacobian[i];
        for (int j=rigid_f*4;j<rigid_f*4+4;++j)
        {
          retval[j+robot_kdl_.dof*(i)]+=finger_gradient[j-rigid_f*4];
        }
      }

      // forward gradient:
      acc_cost=wi*(r_f_poses[i+2+2]-2.0*r_f_poses[i+1+2]+r_f_poses[i+2]);
      acc_cost[3]=0.0;
      acc_cost[4]=0.0;
      acc_cost[5]=0.0;

      if(i<timesteps_-1)
      {
        finger_gradient=2.0*acc_cost.transpose()*r_f_Jacobian[i];
        for (int j=rigid_f*4;j<rigid_f*4+4;++j)
        {
          retval[j+robot_kdl_.dof*(i)]+=finger_gradient[j-rigid_f*4];
        }
      }

      if(i<timesteps_-3)
      {
	finger_gradient=2.0*acc_cost.transpose()*r_f_Jacobian[i+2];
	for (int j=rigid_f*4;j<rigid_f*4+4;++j)
	{
	  retval[j+robot_kdl_.dof*(i+2)]+=finger_gradient[j-rigid_f*4];
	}
      }
      if(i<timesteps_-2)
      {
        finger_gradient=-4.0*acc_cost.transpose()*r_f_Jacobian[i+1];
        for (int j=rigid_f*4;j<rigid_f*4+4;++j)
	{
	  retval[j+robot_kdl_.dof*(i+1)]+=finger_gradient[j-rigid_f*4];
	}

      }


      
    }
    for (int j=0;j<robot_kdl_.dof;++j)
    {
      retval[j+robot_kdl_.dof*(i)]+=gradient[j];
    }
    fill(gradient.begin(), gradient.end(), 0.0);
 

  }

  return retval;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  
  // limiting velocity at joints
  vector<double> vel_const(m_nic,0.0);

  for(int j=0;j<robot_kdl_.dof;++j)
  {
    vel_const[(0)*robot_kdl_.dof+j]=std::abs(x[(0)*robot_kdl_.dof+j]-joints_0[j])-vel_lim;
  }

  for (int i=0; i<timesteps_-1; ++i)
  {
    for(int j=0;j<robot_kdl_.dof;++j)
    {
      vel_const[(i+1)*robot_kdl_.dof+j]=std::abs(x[(i+1)*robot_kdl_.dof+j]-x[(i)*robot_kdl_.dof+j])-vel_lim;
    }
  }

  
  return vel_const;
}

vector<double> optProblem::inEqGradient(vector<double> x) const
{
  // Full gradient:
  /*
  vector<double> retval(m_dim*m_nic,0.0);

  for(int j=0;j<robot_kdl_.dof;++j)
  {
    if(std::abs(x[j]-joints_0[j])>vel_lim)
    {
      if(x[j]-joints_0[j]<0.0)
      {
        retval[j*m_dim+(0)*robot_kdl_.dof+j]=-1.0;
      }
      else
      {
        retval[j*m_dim+(0)*robot_kdl_.dof+j]=1.0;
      }
    }
  }
  

  
  for(int i=0;i<timesteps_-1;++i)
  {
      for(int j=0;j<robot_kdl_.dof;++j)
      {
        if(std::abs(x[(i+1)*robot_kdl_.dof+j]-x[(i)*robot_kdl_.dof+j])>vel_lim)
        {
          if(x[(i+1)*robot_kdl_.dof+j]-x[(i)*robot_kdl_.dof+j]<0.0)
          {
            retval[((i+1)*robot_kdl_.dof+j)*m_dim+(i+1)*robot_kdl_.dof+j]=-1.0;
            retval[((i+1)*robot_kdl_.dof+j)*m_dim+(i)*robot_kdl_.dof+j]=1.0;
          }
          else
          {
            retval[((i+1)*robot_kdl_.dof+j)*m_dim+(i+1)*robot_kdl_.dof+j]=1.0;
            retval[((i+1)*robot_kdl_.dof+j)*m_dim+(i)*robot_kdl_.dof+j]=-1.0;
          }
        }
      }
  }
  */
  vector<double> retval(robot_kdl_.dof+2*(m_nic-robot_kdl_.dof),0.0);

  
  for(int j=0;j<robot_kdl_.dof;++j)
  {
    if(std::abs(x[j]-joints_0[j])>vel_lim)
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
  }
  

  
  for(int i=0;i<timesteps_-1;++i)
  {
      for(int j=0;j<robot_kdl_.dof;++j)
      {
        if(std::abs(x[(i+1)*robot_kdl_.dof+j]-x[(i)*robot_kdl_.dof+j])>vel_lim)
        {
          if(x[(i+1)*robot_kdl_.dof+j]-x[(i)*robot_kdl_.dof+j]<0.0)
          {
            // retval[(i+1)*robot_kdl_.dof+j]
            // retval[i*robot_kdl_.dof+j]
            retval[(i+1)*robot_kdl_.dof+j]=-1.0;
            retval[(i)*robot_kdl_.dof+j]=1.0;
          }
          else
          {
            retval[(i+1)*robot_kdl_.dof+j]=1.0;
            retval[(i)*robot_kdl_.dof+j]=-1.0;
          }
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
    up_bounds.insert(up_bounds.end(), robot_kdl_.up_bounds.begin(), robot_kdl_.up_bounds.end());
    low_bounds.insert(low_bounds.end(), robot_kdl_.low_bounds.begin(), robot_kdl_.low_bounds.end());
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

  for(int i=0;i<ineq_grad.size();i++)
  {
    grad.emplace_back(ineq_grad[i]);
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

  for(int i=0;i<robot_kdl_.dof;++i)
  {
    retval.emplace_back(1+m_nec+i,i);
  }

  // Other timesteps:
  for (int i=robot_kdl_.dof;i<m_nic;++i)
  {
    retval.emplace_back(i+m_nec+1,i-robot_kdl_.dof);
    retval.emplace_back(i+m_nec+1,i);
  }

  return retval;
}

