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
  m_dim=robot_kdl.dof+6;// [robot_dof, obj_pose]
  cerr<<m_dim<<endl;

  m_nic=4;// link collisions with object
  timesteps_=time_steps;
  robot_kdl_=robot_kdl;
  des_poses.resize(1);
  Map<Eigen::VectorXd> des_pose(&data_arr[0][0],3);// only positio
  des_poses[0]=des_pose;
  joints_0=init_joint_angles;
  if(time_steps>1)
  {
    vel_lim=vel_limits[0];
  }
  data_arr_=data_arr;
  m_nec=data_arr_[1].size();
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
  //cost_val+=std::abs(f_poses[3][1]-(f_poses[0][1]+f_poses[1][1]+f_poses[2][1])/3.0);


  //cerr<<cost_val<<endl;
  return cost_val;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  vector<double> ret(m_nic,0.0);
  vector<double> deb_ret(m_nic,0.0);
  vector<vector<double>> r_poses;
  vector<int> link_idx={1,2,4,5,7,8,10,11};
  vector<int> kdl_link_idx={0,1,3,4,6,7,9,10};
  vector<int> f_idx={0,0,1,1,2,2,3,3};
  vector<int> joint_idx_low={0,0,4,4,8,8,12,12};
  vector<int> joint_idx_up={2,3,6,7,10,11,15,16};
  vector<double> l_pose_arr;
  vector<double> q(&x[0],&x[robot_kdl_.dof]);
  for(int i=0;i<f_idx.size();i++)
  {
    vector<double> f_q(&q[joint_idx_low[i]],&q[joint_idx_up[i]]);

    l_pose_arr=robot_kdl_.getLinkPose(f_q,kdl_link_idx[i]);
    
    r_poses.push_back(l_pose_arr);
  }
  vector<vector<collision_data>> c_data=c_checker.check_collisions(r_poses,link_idx);

  for (int j=0;j<c_data.size();++j)
  {
   
    for (int i=0;i<c_data[j].size();++i)
    {
      if(c_data[j][i].collision)
      {
        ret[f_idx[j]]+=c_data[j][i].distance;
      }
    }
  }
  cerr<<ret[0]<<' '<<ret[1]<<' '<<ret[2]<<' '<<ret[3]<<endl;
  return  ret;


  
  
}
vector<double> optProblem::EqConstraints(vector<double> x) const
{
  // no constraint
  vector<double> ret(m_nec,0.0);
  vector<double> deb_ret(m_nec,0.0);
  vector<vector<double>> r_poses;
  vector<int> link_idx={3,6,9,12};
  vector<double> ee_pose_arr;
  // contact constraint:
  for(int i=0;i<data_arr_[1].size();++i)
  {
    int finger=int(data_arr_[1][i]);// rigid_finger
    vector<double> q(&x[0],&x[robot_kdl_.dof]);
    vector<double> f_q(&q[finger*robot_kdl_.f_dof],&q[finger*robot_kdl_.f_dof+robot_kdl_.f_dof]);
    ee_pose_arr=robot_kdl_.getFK(finger,f_q,false);
    r_poses.push_back(ee_pose_arr);
    
  }
  vector<vector<collision_data>> c_data=c_checker.check_collisions(r_poses,link_idx);

  for (int j=0;j<c_data.size();++j)
  {
    double min_dist=std::abs(c_data[j][0].distance);
    int idx=0;
    for (int i=0;i<c_data[j].size();++i)
    {
      if(c_data[j][i].distance>data_arr_[4][0] || c_data[j][i].distance<data_arr_[4][0])
      {
        if(std::abs(c_data[j][i].distance)<min_dist)
        {
          min_dist=std::abs(c_data[j][i].distance);
          idx=i;
        }
      }
    }
    if(c_data[j][idx].distance>data_arr_[4][0] || c_data[j][idx].distance<data_arr_[4][0])
    {
      ret[j]+=min_dist;
      deb_ret[j]=c_data[j][idx].distance;
    }
  }
  
  //cerr<<deb_ret[0]<<' '<<deb_ret[1]<<' '<<deb_ret[2]<<' '<<deb_ret[3]<<endl;
  return  ret;
  
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  vector<double> retval(m_dim,0.0);
  
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
  double pos_err=f_poses[3][1]-(f_poses[0][1]+f_poses[1][1]+f_poses[2][1])/3.0;
  Eigen::VectorXd finger_gradient;
  vector<double> gradient;
  gradient.resize(robot_kdl_.dof,0.0);
  Eigen::VectorXd pose_err(3);
  pose_err.setZero();
  pose_err[1]=pos_err;
  for(int i=0;i<data_arr_[1].size();++i)
  {
    vector<double> q(&x[0],&x[robot_kdl_.dof]);    
    int finger=int(data_arr_[1][i]);// rigid_finger
    vector<double> f_q(&q[finger*robot_kdl_.f_dof],&q[finger*robot_kdl_.f_dof+robot_kdl_.f_dof]);
    if(i!=3)
    {
      finger_gradient=-1.0*pose_err.transpose()*robot_kdl_.getJacobian(finger,f_q).block(0,0,3,4);
    }
    else
    {
      finger_gradient=1.0*pose_err.transpose()*robot_kdl_.getJacobian(finger,f_q).block(0,0,3,4);
    }
    for (int k=0;k<robot_kdl_.f_dof;++k)
    {
      gradient[finger*robot_kdl_.f_dof+k]+=finger_gradient[k];
    }
  } 
  for (int j=0;j<robot_kdl_.dof;++j)
  {
    //retval[j]+=gradient[j];
  }
  return retval;
}

vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nic,0.0);

  vector<vector<double>> r_poses;
  vector<int> link_idx={1,2,4,5,7,8,10,11};
  vector<int> kdl_link_idx={0,1,3,4,6,7,9,10};
  vector<int> f_idx={0,0,1,1,2,2,3,3};
  vector<int> joint_idx_low={0,0,4,4,8,8,12,12};
  vector<int> joint_idx_up={2,3,6,7,10,11,15,16};
  //vector<int> joint_idx_up={2,3,6,7,10,11,15,16};
  vector<double> l_pose_arr;
  vector<double> q(&x[0],&x[robot_kdl_.dof]);
  for(int i=0;i<f_idx.size();i++)
  {
    vector<double> f_q(&q[joint_idx_low[i]],&q[joint_idx_up[i]]);
    l_pose_arr=robot_kdl_.getLinkPose(f_q,kdl_link_idx[i]);
    r_poses.push_back(l_pose_arr);
  }
  //cerr<<r_poses[1][0]<<endl;
  vector<vector<collision_data>> c_data=c_checker.check_collisions(r_poses,link_idx);

  Eigen::VectorXd finger_gradient(4);
  finger_gradient.setZero();
  //vector<int> joint_idx_up_={1,2,5,6,9,10,14,15};  
  for (int j=0;j<f_idx.size();++j)
  {

    vector<double> f_q(&q[joint_idx_low[j]],&q[joint_idx_up[j]]);
    Eigen::MatrixXd link_J=robot_kdl_.getLinkJacobian(kdl_link_idx[j],f_q).block(0,0,3,f_q.size());
    Map<Eigen::VectorXd> link_pose(&r_poses[j][0],3);
    for (int i=0;i<c_data[j].size();++i)
    {
      if(c_data[j][i].collision)
      {
        Map<Eigen::VectorXd> point(&c_data[j][i].point[0],3);
        Map<Eigen::VectorXd> dir(&c_data[j][i].dir[0],3);
        double p_d;
        p_d=c_data[j][i].distance;
        Eigen::VectorXd delta_p;        
        delta_p=link_pose-(point-p_d*dir);
        finger_gradient.segment(0,f_q.size())+=delta_p.transpose()*link_J;
      }
    }
    // add to gradient:
    for(int k=0;k<robot_kdl_.f_dof;++k)
    {
      retval[f_idx[j]*m_dim+f_idx[j]*robot_kdl_.f_dof+k]=finger_gradient[k];
    }
    
  }
  return retval;
}

vector<double> optProblem::eqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nec,0.0);
  vector<vector<double>> r_poses;
  vector<int> link_idx={3,6,9,12};
  vector<double> ee_pose_arr;

  Eigen::VectorXd finger_gradient(4);
  finger_gradient.setZero();
  // contact constraint:
  for(int i=0;i<data_arr_[1].size();++i)
  {
    int finger=int(data_arr_[1][i]);// rigid_finger
    vector<double> q(&x[0],&x[robot_kdl_.dof]);
    vector<double> f_q(&q[finger*robot_kdl_.f_dof],&q[finger*robot_kdl_.f_dof+robot_kdl_.f_dof]);
    ee_pose_arr=robot_kdl_.getFK(finger,f_q,false);
    r_poses.push_back(ee_pose_arr);
    
  }
  vector<vector<collision_data>> c_data=c_checker.check_collisions(r_poses,link_idx);

  for (int j=0;j<c_data.size();++j)
  {
    //int j=2;
    // Get link jacobian:
    int finger=int(data_arr_[1][j]);// rigid_finger
    vector<double> q(&x[0],&x[robot_kdl_.dof]);
    vector<double> f_q(&q[finger*robot_kdl_.f_dof],&q[finger*robot_kdl_.f_dof+robot_kdl_.f_dof]);

    Eigen::MatrixXd link_J=robot_kdl_.getJacobian(finger,f_q).block(0,0,3,4);
    // Get link pose:

    
    Map<Eigen::VectorXd> link_pose(&r_poses[j][0],3);
    finger_gradient.setZero();
    double min_dist=std::abs(c_data[j][0].distance);
    int idx=0;
    for (int i=0;i<c_data[j].size();++i)
    {
      if(c_data[j][i].distance>data_arr_[4][0] || c_data[j][i].distance<data_arr_[4][0])
      {
        if(std::abs(c_data[j][i].distance)<min_dist)
        {
          min_dist=std::abs(c_data[j][i].distance);
          idx=i;
        }
      }
    }
    // Get distance to move
    Map<Eigen::VectorXd> point(&c_data[j][idx].point[0],3);
    Map<Eigen::VectorXd> dir(&c_data[j][idx].dir[0],3);
    double p_d;
    p_d=c_data[j][idx].distance;
    Eigen::VectorXd delta_p;
    
    delta_p=link_pose-(point-p_d*dir);
    if(c_data[j][idx].distance<data_arr_[4][0]|| c_data[j][idx].distance>data_arr_[4][0])
    {
      finger_gradient=delta_p.transpose()*link_J;
    }
    // Mulltiply with jacobian to compute joint gradient

    // add to arg gradient
    for(int k=0;k<robot_kdl_.f_dof;++k)
    {
      retval[j*m_dim+j*robot_kdl_.f_dof+k]=finger_gradient[k];
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

  for(int i=0;i<m_nec;++i)
  {
    for(int j=0;j<robot_kdl_.f_dof;++j)
    {
      grad.emplace_back(eq_grad[i*m_dim+i*robot_kdl_.f_dof+j]);
    }     
  }

  for(int i=0;i<m_nic;++i)
  {
    for(int j=0;j<robot_kdl_.f_dof;++j)
    {
      grad.emplace_back(ineq_grad[i*m_dim+i*robot_kdl_.f_dof+j]);
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
    for (int j=0;j<robot_kdl_.f_dof;++j)
    {
      retval.emplace_back(i+1,i*robot_kdl_.f_dof+j);
    }
  }
  // inequality constraints:
  for (int i=0;i<m_nic;++i)
  {
    for(int j=0;j<robot_kdl_.f_dof;++j)
    {
      retval.emplace_back(i+1+m_nec,i*robot_kdl_.f_dof+j);
    }
  }

  return retval;
}
vector<vector<double>> optProblem::bounds() const
{
  vector<vector<double>> bound;
  vector<double> up_bounds;
  vector<double> low_bounds;
  //vector<double> kdl_up_bounds=robot_kdl_.up_bounds;
  //vector<double> kdl_low_bounds=robot_kdl_.low_bounds;
  vector<double> obj_pose_up_bounds=data_arr_[2];
  vector<double> obj_pose_low_bounds=data_arr_[3];
  
  up_bounds.insert(up_bounds.end(),
                   robot_kdl_.up_bounds.begin(),
                   robot_kdl_.up_bounds.end());
  up_bounds.insert(up_bounds.end(),
                   obj_pose_up_bounds.begin(),
                   obj_pose_up_bounds.end());
  low_bounds.insert(low_bounds.end(),
                   robot_kdl_.low_bounds.begin(),
                   robot_kdl_.low_bounds.end());
  low_bounds.insert(low_bounds.end(),
                   obj_pose_low_bounds.begin(),
                   obj_pose_low_bounds.end());
  
 
  bound.resize(2);
  bound[0]=low_bounds;
  bound[1]=up_bounds;
  return bound;
  
}



