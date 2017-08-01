#include "optimization_pkg/problems/opt_problem.hpp"

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
optProblem::optProblem(int time_steps,manipulator_kdl::robotKDL robot_kdl,vector<vector<double>> des_poses_,vector<double> vel_limits, vector<double> init_joint_angles)
{
  m_dim=robot_kdl.dof*time_steps;
  m_nec=robot_kdl.dof;//robot_kdl.dof*time_steps;// no equality constraints
  m_nic=robot_kdl.dof*(time_steps-1);// velocity constraints
  timesteps_=time_steps;
  robot_kdl_=robot_kdl;
  des_poses.resize(2);
  joints_0=init_joint_angles;
  vel_lim=vel_limits[0];
  for(int i=0;i<2;++i)
  {    
    Map<Eigen::VectorXd> des_pose(&des_poses_[0][0],des_poses_[0].size());// only position
    des_poses[i]=des_pose;
  }
}

double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;
  vector<double> q;
  vector<double> ee_pose_arr;
  Eigen::VectorXd pos_err;


  q.resize(robot_kdl_.dof,0.0);
  for(int i=1; i<timesteps_;++i)
  {
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }
    vector<double> f_q(&q[3*4],&q[3*4+4]);

    ee_pose_arr=robot_kdl_.getFK(3,f_q,true);
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);// only position
    
    pos_err=des_poses[0]-ee_pose;
    pos_err[3]=0.0;
    pos_err[4]=0.0;
    pos_err[5]=0.0;

    //if(i==timesteps_-1)
    {  
      cost_val+=pos_err.lpNorm<1>();
    }
  }
  //cerr<<cost_val<<endl;
  return cost_val;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  // limiting velocity at joints
  vector<double> vel_const(m_nic,0.0);
  for (int i=0; i<timesteps_-1; ++i)
  {
    for(int j=0;j<robot_kdl_.dof;++j)
    {
      vel_const[(i)*robot_kdl_.dof+j]=std::abs(x[(i+1)*robot_kdl_.dof+j]-x[(i)*robot_kdl_.dof+j])-vel_lim;
    }
  }
  //cerr<<vel_const[1]<<endl;
  return vel_const;
}
vector<double> optProblem::EqConstraints(vector<double> x) const
{
  // no constraint
  vector<double> ret(m_nec,0.0);
  // Equality constraint to ensure initial joint angles
  for (int i=0;i<16;++i)
  {
    ret[i]=joints_0[i]-x[i];
    //cerr<<ret[i]<<endl;
  }
  
  return  ret;
  
}

vector<double> optProblem::objGradient(vector<double> x) const
{
  vector<double> retval(m_dim,0.0);
  vector<double> q;
  vector<double> ee_pose_arr;
  Eigen::VectorXd pos_err,pose_err;
  Eigen::VectorXd gradient,finger_gradient;
  gradient.resize(robot_kdl_.dof);
  finger_gradient.resize(4);// finger dof
  pose_err.resize(6);
  q.resize(robot_kdl_.dof,0.0);

  //int i=timesteps_-1;
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }
    vector<double> f_q(&q[3*4],&q[3*4+4]);

    
    ee_pose_arr=robot_kdl_.getFK(3,f_q,true);
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);// only position
    
    pos_err=des_poses[0]-ee_pose;

    pose_err=pos_err;
    pose_err[3]=0.0;
    pose_err[4]=0.0;
    pose_err[5]=0.0;
    //pose_err[3]=pose_err[3]*-1.0;
    //pose_err[4]=pose_err[4]*-1.0;

    finger_gradient=-1.0*pose_err.transpose()*robot_kdl_.getJacobian(3,f_q);
    for (int k=0;k<4;++k)
    {
      gradient[3*4+k]=finger_gradient[k];
    }
    for (int j=0;j<robot_kdl_.dof;++j)
    {
      retval[j+robot_kdl_.dof*(i)]=gradient[j];
    }
  }
  
  return retval;
}

vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nic,0.0);
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
  return retval;
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
        retval[i*m_dim+j]=-1.0;
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
vector<vector<double>> optProblem::bounds() const
{
  vector<vector<double>> bound;
  vector<double> up_bounds;
  vector<double> low_bounds;
  low_bounds.resize(robot_kdl_.dof*timesteps_,-0.5);

  up_bounds.resize(robot_kdl_.dof*timesteps_,0.5);

  bound.resize(2);
  bound[0]=low_bounds;
  bound[1]=up_bounds;
  return bound;
  
}


