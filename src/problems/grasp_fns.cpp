#include "optimization_pkg/problems/opt_problem.hpp"

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
optProblem::optProblem(int time_steps, manipulator_kdl::robotKDL robot_kdl,
                       vector<vector<double>> data_arr,vector<double> vel_limits,
                       vector<double> init_joint_angles)
{
  cerr<<"Called wrong constructor!!"<<endl;
}

optProblem::optProblem(int time_steps,manipulator_kdl::robotKDL robot_kdl,collisionChecker c_check, vector<vector<double>> data_arr,vector<double> vel_limits,vector<double> init_joint_angles)
{
  m_dim=robot_kdl.dof;//+6;//+6;// [robot_dof, obj_pose]
  //cerr<<m_dim<<endl;

  m_nic=4;// link collisions with object
  timesteps_=time_steps;
  robot_kdl_=robot_kdl;
  des_poses.resize(1);
  Map<Eigen::VectorXd> des_pose(&data_arr[0][0],3);// only position
  des_poses[0]=des_pose;
  joints_0=init_joint_angles;
  if(time_steps>1)
  {
    vel_lim=vel_limits[0];
  }
  data_arr_=data_arr;
  m_nec=0;//data_arr_[1].size();
  c_checker=c_check;
  
  cerr<<"got opt parameters "<<m_dim<<" "<<m_nec<<" "<<m_nic<<endl;
}


double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;
  vector<Eigen::VectorXd> f_poses;
  vector<double> ee_pose_arr;
  for(int i=0;i<data_arr_[1].size();++i)
  {
    int finger=int(data_arr_[1][i]);// rigid_finger
    vector<double> q(&x[0],&x[robot_kdl_.dof]);    

    
    vector<double> f_q(&q[finger*robot_kdl_.f_dof],&q[finger*robot_kdl_.f_dof+robot_kdl_.f_dof]);
    
    ee_pose_arr=robot_kdl_.getFK(finger,f_q,true);
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],3);
    f_poses.push_back(ee_pose);
  }

  // y cost:
  //cost_val+=std::abs(f_poses[3][1]-f_poses[1][1]);//+std::abs(f_poses[1][1]-f_poses[2][1]-0.05);
  
  //cerr<<"x "<<x.size()<<endl;

  return cost_val;
}
vector<double> optProblem::EqConstraints(vector<double> x) const
{
  // no constraint
  vector<double> ret(m_nec,0.0);
  
  return  ret;
  
}

vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  vector<double> ret(m_nic,0.0);
  return  ret;
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  vector<double> retval(m_dim,0.0);
  return retval;
}


vector<double> optProblem::eqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nec,0.0);
  return retval;
}
vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nic,0.0);
  return retval;
}

vector<double> optProblem::sparse_gradient(const vector<double> x) const
{
  vector<double> grad;
  vector<double> obj_grad=objGradient(x);
  vector<double> eq_grad=eqGradient(x);
  vector<double> ineq_grad=inEqGradient(x);
  
  return grad;
}
std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> optProblem::gradient_sparsity() const
{
  std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> retval;
  
  return retval;
}
vector<vector<double>> optProblem::bounds() const
{
  vector<vector<double>> bound;
  vector<double> up_bounds;
  vector<double> low_bounds;
  //vector<double> kdl_up_bounds=robot_kdl_.up_bounds;
  //vector<double> kdl_low_bounds=robot_kdl_.low_bounds;
  
  up_bounds.insert(up_bounds.end(),
                   robot_kdl_.up_bounds.begin(),
                   robot_kdl_.up_bounds.end());
  //up_bounds.insert(up_bounds.end(),obj_pose_up_bounds.begin(), obj_pose_up_bounds.end());
  low_bounds.insert(low_bounds.end(),
                   robot_kdl_.low_bounds.begin(),
                   robot_kdl_.low_bounds.end());
  //low_bounds.insert(low_bounds.end(),obj_pose_low_bounds.begin(),obj_pose_low_bounds.end());
  bound.resize(2);
  bound[0]=low_bounds;
  bound[1]=up_bounds;
  return bound;
  
}



