//#define SPARSE_GRADIENT true
#include <in_hand_manipulation/problems/opt_problem.hpp>

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
optProblem::optProblem(int time_steps, manipulator_kdl::robotKDL &robot_kdl,vector<vector<double>> data_arr,vector<double> vel_limits, vector<double> init_joint_angles)
{
  m_dim=robot_kdl.dof;
  
  m_nec=0;// no check to match initial joint angles
  m_nic=0;// velocity constraints
  //timesteps_=time_steps;
  robot_kdl_=&robot_kdl;
  des_poses.resize(robot_kdl_->dof+1);
  joints_0=init_joint_angles;
  //vel_lim=vel_limits[0];
  data_arr_=data_arr;
  
  for(int i=0;i<4+1;++i)
  {    
    Map<Eigen::VectorXd> des_pose(&data_arr[i][0],data_arr[i].size());// only position
    des_poses[i]=des_pose;
  }
}

double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose
  int n_f=4;
  double cost_val=0.0;
  Eigen::VectorXd f_pos_err;
  double position_cost=0.0;
  Eigen::VectorXd ee_pose;
  // Getting joint angles for current timestep:
  Map<const Eigen::VectorXd> q(&x[0], x.size());
  for (int f=0;f<n_f;f++)
  {
    if(des_poses[n_f][f]==1)// check if fingertip is in f_index
    {
      robot_kdl_->getFK(f,q.segment(f*4,4),ee_pose,true);
      // Finger pose error:
      f_pos_err=des_poses[f]-ee_pose.head(3);
      cost_val+=f_pos_err.squaredNorm();
    }
  }    
  
  
  return cost_val;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  // limiting velocity at joints
  vector<double> vel_const(m_nic,0.0);
  // Since IK, no velocity constraint
  return vel_const;
}

vector<double> optProblem::EqConstraints(vector<double> x) const
{
  // no constraint
  vector<double> ret(m_nec,0.0);  
  return  ret;
  
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  vector<double> retval(m_dim,0.0);
  //Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],3);
  const Map<Eigen::VectorXd> q(&x[0],x.size());
  vector<double> ee_pose_arr;
  Eigen::VectorXd ee_pose;
  Eigen::VectorXd f_pos_err;
  Eigen::VectorXd f_pose_err;
  Eigen::VectorXd finger_gradient;
  Eigen::MatrixXd J;
  vector<double> gradient;
  gradient.resize(robot_kdl_->dof,0.0);

  f_pos_err.resize(3);
  f_pose_err.resize(6);
  int n_f=4;
  for (int f=0;f<n_f;f++)
  {
    if(des_poses[n_f][f]==1)
    { 
      robot_kdl_->getFK(f,q.segment(f*4,4),ee_pose,true);
    
      f_pos_err=des_poses[f]-ee_pose.head(3);
      f_pose_err[0]=f_pos_err[0];
      f_pose_err[1]=f_pos_err[1];
      f_pose_err[2]=f_pos_err[2];
      f_pose_err[3]=0.0;
      f_pose_err[4]=0.0;
      f_pose_err[5]=0.0;
      robot_kdl_->getJacobian(f,q.segment(f*4,4),J);
      finger_gradient=-2.0*f_pose_err.transpose()*J;
      
      for (int k=0;k<4;++k)
      {
        gradient[f*4+k]+=finger_gradient[k];
      }
    }
  }  
  for (int j=0;j<robot_kdl_->dof;++j)
  {
    retval[j]+=gradient[j];
  }
  return retval;
}

vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nic,0.0);
  
  return retval;
}

vector<double> optProblem::eqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nec,0.0);
  
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
    retval.emplace_back(i+m_nec+1,i+robot_kdl_->dof);
    
  }
  return retval;
}
*/
vector<vector<double>> optProblem::bounds() const
{
  vector<vector<double>> bound;
  vector<double> up_bounds;
  vector<double> low_bounds;

  up_bounds.insert(up_bounds.end(), robot_kdl_->up_bounds.begin(), robot_kdl_->up_bounds.end());
  low_bounds.insert(low_bounds.end(), robot_kdl_->low_bounds.begin(), robot_kdl_->low_bounds.end());
  
  
  bound.resize(2);
  bound[0]=low_bounds;
  bound[1]=up_bounds;
  return bound;
  
}


