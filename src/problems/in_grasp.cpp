#include "optimization_pkg/problems/opt_problem_full.hpp"

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
optProblem::optProblem(int time_steps,manipulator_kdl::robotKDL robot_kdl,vector<vector<double>> data_arr,vector<double> vel_limits, vector<double> init_joint_angles)
{
  m_dim=robot_kdl.dof*time_steps;
  m_nec=robot_kdl.dof;//robot_kdl.dof*time_steps;// no equality constraints
  m_nic=robot_kdl.dof*(time_steps-1);// velocity constraints
  timesteps_=time_steps;
  robot_kdl_=robot_kdl;
  des_poses.resize(2);
  joints_0=init_joint_angles;
  vel_lim=vel_limits[0];
  data_arr_=data_arr;
  
  for(int i=0;i<1;++i)
  {    
    Map<Eigen::VectorXd> des_pose(&data_arr[0][0],data_arr[0].size());// only position
    des_poses[i]=des_pose;
  }
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
  
  double wo,wr,wd;
  wo=data_arr_[1][0];
  wr=data_arr_[1][1];
  wd=data_arr_[1][2];
  
  q.resize(robot_kdl_.dof,0.0);
  for(int i=1; i<timesteps_;++i)
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


    // object pose error:
    
    obj_pos_err=des_poses[0]-ee_pose;

    obj_pos_err[0]=obj_pos_err[0];
    obj_pos_err[1]=obj_pos_err[1];
    obj_pos_err[2]=obj_pos_err[2];
    
    obj_pos_err[3]=0.0;
    obj_pos_err[4]=0.0;
    obj_pos_err[5]=0.0;

    // Relaxed-Rigidity costs:
    
    for(int k=0;k<data_arr_[3].size();++k)
    {
      int f=data_arr_[3][k];
      if(f!=rigid_f)
      {
        // Position:
        vector<double> f_joints(&q[f*4],&q[f*4+4]);
        
        vector<double> f_pose_arr=robot_kdl_.getFK(f,f_joints,true);
        Map<Eigen::VectorXd> f_pose(&f_pose_arr[0],f_pose_arr.size());
        
        Eigen::VectorXd pose_diff=ee_pose-f_pose-pos_offsets[k];
        pos_err=pose_diff.segment(0,3);
        orient_err=pose_diff.segment(3,3);
        orient_err[0]=0.0;
        orient_err[2]=0.0;
        position_cost+=pos_err.lpNorm<1>();
        orientation_cost+=orient_err.lpNorm<1>();
      }
    }
    
    if(i==timesteps_-1)
    { 
      cost_val+=wd*obj_pos_err.lpNorm<1>();//+wr*position_cost+wo*orientation_cost;
      //cerr<<cost_val<<endl;
    }
    else
    {
      //cost_val+=wr*position_cost+wo*orientation_cost;
    }
    
  }
  
  cerr<<cost_val<<endl;
  return cost_val;
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  double wo,wr,wd;
  wo=data_arr_[1][0];
  wr=data_arr_[1][1];
  wd=data_arr_[1][2];
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


  // position offsets:
  vector<Eigen::VectorXd> pos_offsets;
  
  for(int k=0;k<data_arr_[3].size();++k)
  {
    vector<double> p_off=data_arr_[4+k];
    Map<Eigen::VectorXd> pos_offset(&p_off[0],p_off.size());
    pos_offsets.emplace_back(pos_offset);
  }
  

  

  for(int i=1;i<timesteps_;++i)
  {
    
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }
    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);

    
    ee_pose_arr=robot_kdl_.getFK(rigid_f,f_q,true);
    
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);// only position

    obj_pose_err=des_poses[0]-ee_pose;

    obj_pose_err[0]=obj_pose_err[0];
    obj_pose_err[1]=obj_pose_err[1];
    obj_pose_err[2]=obj_pose_err[2];
    
    obj_pose_err[3]=0.0;
    obj_pose_err[4]=0.0;
    obj_pose_err[5]=0.0;

    if(i==timesteps_-1)
    {
      finger_gradient=-1.0*wd*obj_pose_err.transpose()*robot_kdl_.getJacobian(rigid_f,f_q);
      cerr<<finger_gradient.transpose()<<endl;
      for (int k=0;k<4;++k) 
      {
        gradient[rigid_f*4+k]+=finger_gradient[k];
      }
    }
    // Relaxed-Rigidity costs:

    for(int k=0;k<data_arr_[3].size();++k)
    {
      int f=data_arr_[3][k];
      if(f!=rigid_f)
      {
        // Position:
        vector<double> f_joints(&q[f*4],&q[f*4+4]);
        
        vector<double> f_pose_arr=robot_kdl_.getFK(f,f_joints,true);
        Map<Eigen::VectorXd> f_pose(&f_pose_arr[0],f_pose_arr.size());
        
        Eigen::VectorXd pose_diff=ee_pose-f_pose-pos_offsets[k];
        pos_err=pose_diff;//.segment(0,3);
        //orient_err=pose_diff;//.segment(3,3);
        pos_err[0]=wr*pos_err[0];
        pos_err[1]=wr*pos_err[1];
        pos_err[2]=wr*pos_err[2];
        pos_err[3]=0.0;
        pos_err[4]=-1.0*pos_err[4]*wo;
        pos_err[5]=0.0;
        
        //finger_gradient=-1.0*pos_err.transpose()*robot_kdl_.getJacobian(f,f_joints);
        for (int l=0;l<4;++l)
        {
          //gradient[f*4+l]+=finger_gradient[l];
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
  /*
  for (int i=0; i<timesteps_-1; ++i)
  {
    for(int j=0;j<robot_kdl_.dof;++j)
    {
      vel_const[(i)*robot_kdl_.dof+j]=std::abs(x[(i+1)*robot_kdl_.dof+j]-x[(i)*robot_kdl_.dof+j])-vel_lim;
    }
  }
  */
  //cerr<<vel_const[1]<<endl;
  return vel_const;
}

vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nic,0.0);
  /*
  for(int i=0;i<timesteps_-1;++i)
  {
      for(int j=0;j<robot_kdl_.dof;++j)
      {

        if(x[(i+1)*robot_kdl_.dof+j]-x[(i)*robot_kdl_.dof+j]<0.0)
        {
          retval[((i)*robot_kdl_.dof+j)*m_dim+(i+1)*robot_kdl_.dof+j]=-1.0;
          retval[((i)*robot_kdl_.dof+j)*m_dim+(i)*robot_kdl_.dof+j]=1.0;
        }
        else
        {
          retval[((i)*robot_kdl_.dof+j)*m_dim+(i+1)*robot_kdl_.dof+j]=1.0;
          retval[((i)*robot_kdl_.dof+j)*m_dim+(i)*robot_kdl_.dof+j]=-1.0;
        }

      }
  }
  */
  return retval;
}
vector<double> optProblem::EqConstraints(vector<double> x) const
{
  // no constraint
  vector<double> ret(m_nec,0.0);
  // Equality constraint to ensure initial joint angles
  for (int i=0;i<robot_kdl_.dof;++i)
  {
    //ret[i]=joints_0[i]-x[i];
    //cerr<<ret[i]<<endl;
  }
  
  return  ret;
  
}

vector<double> optProblem::eqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nec,0.0);
  // gradient:
  for(int i=0;i<robot_kdl_.dof;++i)
  {
    for (int j=0;j<robot_kdl_.dof;++j)
    {
      if(i==j)
      {
        //retval[i*m_dim+j]=-1.0;
      }
    }
  }
  
  return retval;
}

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

/*

vector<double> optProblem::sparse_gradient(const vector<double> x) const
{
  vector<double> grad;
  vector<double> obj_grad=objGradient(x);
  vector<double> eq_grad=eqGradient(x);
  vector<double> ineq_grad=inEqGradient(x);
  
  grad.resize(m_dim,0.0);

  for(int i=0;i<m_dim;i++)
  {
    grad[i]=obj_grad[i];
  }

  for(int i=m_dim;i<m_dim+m_dim*m_nec;i++)
  {
    if(eq_grad[i-m_dim]!=0.0)
    {
      grad.emplace_back(eq_grad[i-m_dim]);
    }
  }

  for(int i=m_dim+m_dim*m_nec;i<m_dim+m_dim*m_nec+m_dim*m_nic;i++)
  {
    if(ineq_grad[i-m_dim-m_dim*m_nec]!=0.0)
    {
      grad.emplace_back(ineq_grad[i-m_dim-m_dim*m_nec]);
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
  // Equality constraints:
  for (int i=0;i<m_nec;++i)
  {
    retval.emplace_back(i+1,i);
  }
  // inequality constraints:
  for (int i=0;i<m_nic;++i)
  {
    retval.emplace_back(i+m_nec+1,i);
    retval.emplace_back(i+m_nec+1,i+robot_kdl_.dof);
    
  }
  return retval;
}

*/
