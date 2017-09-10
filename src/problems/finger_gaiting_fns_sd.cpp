#include "optimization_pkg/problems/opt_problem.hpp"
#define COLL_CHECK true
using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
optProblem::optProblem(int time_steps,manipulator_kdl::robotKDL robot_kdl,vector<vector<double>> data_arr,vector<double> vel_limits, vector<double> init_joint_angles)
{
  cerr<<"Wrong constructor!!"<<endl;
}

optProblem::optProblem(int time_steps,manipulator_kdl::robotKDL robot_kdl,collisionChecker c_check, vector<vector<double>> data_arr,vector<double> vel_limits,vector<double> init_joint_angles)
{
  m_dim=robot_kdl.dof*time_steps;
  m_nec=0;// initial joint angle constraint + contact constraint
  m_nic=robot_kdl.dof*(time_steps-1)+1+1+1+1;//+1;// velocity(4) collision(3) stability(1)
  
  //m_nec=robot_kdl.dof;//robot_kdl.dof*time_steps;// no equality constraints
  //m_nic=robot_kdl.dof*(time_steps-1);// velocity constraints

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
  c_checker=c_check;
  
  //cerr<<"got opt parameters "<<m_dim<<" "<<m_nec<<" "<<m_nic<<endl;
}


double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;
  vector<double> q;
  vector<double> ee_pose_arr;
  Eigen::VectorXd f_pos_err;
  
  int finger=int(data_arr_[1][0]);// finger to gait

  
  q.resize(robot_kdl_.dof,0.0);
  
  for(int i=0; i<timesteps_;++i)
  {
    //double position_cost=0.0;
    //double orientation_cost=0.0;
    
    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }
    //vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);
    
    ee_pose_arr=robot_kdl_.getFK(finger,q,true);
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],3);


    // object pose error:
    
    f_pos_err=des_poses[0]-ee_pose;

    
    
    if(i==timesteps_-1)
    { 
      cost_val+=f_pos_err.lpNorm<1>();//+wr*position_cost+wo*orientation_cost;
    }
    
    
  }

  //cerr<<cost_val<<endl;
  return cost_val;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  // limiting velocity at joints
  vector<double> ret(m_nic,0.0);
  if(COLL_CHECK==false)
  {
    return ret;
  }

  int finger=int(data_arr_[1][0]);// rigid_finger
  vector<double> q(robot_kdl_.dof);
  for(int j=0; j<robot_kdl_.dof;++j)
  {
    q[j]=x[robot_kdl_.dof+j];
  }
  
  // Collision constraint between hand and object
  vector<double> ee_pose_arr;
  vector<int> link_idx={1,2,4,5,7,8,10,11};
  vector<int> f_idx;//(2);
  vector<int> joint_idx_low;//={0,0,4,4,8,8,12,12};
  vector<int> joint_idx_up;//={2,3,6,7,10,11,15,16};
  vector<int> kdl_link_idx;//={0,1,3,4,6,7,9,10};
  switch(finger){
    case 0:
      f_idx={1,2};
      joint_idx_low={0,0};
      joint_idx_up={2,3};
      kdl_link_idx={0,1};
      break;
    case 1:
      f_idx={4,5};
      joint_idx_low={0,0};
      joint_idx_up={2,3};
      kdl_link_idx={3,4};
      break;
    case 2:
      f_idx={7,8};
      joint_idx_low={0,0};
      joint_idx_up={2,3};
      kdl_link_idx={6,7};
      break;
    case 3:
      f_idx={10,11};
      joint_idx_low={0,0};
      joint_idx_up={3,4};
      kdl_link_idx={9,10};
      break;
  }
  vector<vector<double>> r_poses;
  for (int i=0;i<2;++i)
  {
    vector<double> f_q(&q[joint_idx_low[i]],&q[joint_idx_up[i]]);
    ee_pose_arr=robot_kdl_.getLinkPose(f_q,kdl_link_idx[i]);//robot_kdl_.getFK(finger,q,false);
    r_poses.push_back(ee_pose_arr);
  }
  vector<vector<collision_data>> c_data=c_checker.check_collisions(r_poses,f_idx);
  
  for (int j=0;j<c_data.size();++j)
  {
    int idx=0;
    for (int i=0;i<c_data[j].size();++i)
    {
      if(c_data[j][i].collision)
      {
        ret[robot_kdl_.dof*(timesteps_-1)]+=c_data[j][i].distance;
      }
    }
  }
  // self collisions:
  vector<int> coll_idx;
  switch(finger){
    case 0:
      f_idx={1,2,3};
      joint_idx_low={0,0,0};
      joint_idx_up={2,3,4};
      kdl_link_idx={0,1,2};
      coll_idx={4,5,6,7,8,9,10,11,12};
      break;
    case 1:
      f_idx={4,5,6};
      joint_idx_low={0,0,0};
      joint_idx_up={2,3,4};
      kdl_link_idx={3,4,5};
      coll_idx={1,2,3,7,8,9,10,11,12};
      break;
    case 2:
      f_idx={7,8,9};
      joint_idx_low={0,0,0};
      joint_idx_up={2,3,4};
      kdl_link_idx={6,7,8};
      coll_idx={1,2,3,4,5,6,10,11,12};
      break;
    case 3:
      f_idx={10,11,12};
      joint_idx_low={0,0,0};
      joint_idx_up={3,4,4};
      kdl_link_idx={9,10,11};
      coll_idx={1,2,3,4,5,6,7,8,9};
      break;
  }  
  r_poses.clear();
  for (int i=0;i<3;++i)
  {
    vector<double> f_q(&q[joint_idx_low[i]],&q[joint_idx_up[i]]);
    ee_pose_arr=robot_kdl_.getLinkPose(f_q,kdl_link_idx[i]);//robot_kdl_.getFK(finger,q,false);
    r_poses.push_back(ee_pose_arr);
  }
  c_data=c_checker.check_self_collisions(r_poses,f_idx,coll_idx);
  //c_checker.publish_robot_objects();
  for (int j=0;j<c_data.size();++j)
  {
    for (int i=0;i<c_data[j].size();++i)
    {
      if(c_data[j][i].collision)
      {
        ret[robot_kdl_.dof*(timesteps_-1)+1]+=c_data[j][i].distance;

      }
    }
  }
  // Contact constraint for fingertip:

  link_idx={3,6,9,12};
  f_idx={link_idx[finger]};
  ee_pose_arr=robot_kdl_.getFK(finger,q,false);
  r_poses.clear();
  r_poses.push_back(ee_pose_arr);
  c_data=c_checker.check_collisions(r_poses,f_idx);
  for (int j=0;j<c_data.size();++j)
  {
    double min_dist=std::abs(c_data[j][0].distance);
    int idx=0;
    for (int i=0;i<c_data[j].size();++i)
    {
      if(!(c_data[j][i].distance>data_arr_[4][0] && c_data[j][i].distance<data_arr_[4][1]))
      {
        if(std::abs(c_data[j][i].distance)<min_dist)
        {
          min_dist=std::abs(c_data[j][i].distance);
          idx=i;
        }
      }
    }
    if(!(c_data[j][idx].distance>data_arr_[4][0] && c_data[j][idx].distance<data_arr_[4][1]))
    {
      ret[robot_kdl_.dof*(timesteps_-1)+1+1]=std::abs(c_data[j][idx].distance)*10.0;      
    }

  }
  return ret;
  }
vector<double> optProblem::EqConstraints(vector<double> x) const
{
  //cerr<<"Eq constraints"<<endl;
  int finger=int(data_arr_[1][0]);// rigid_finger
  // no constraint
  vector<double> ret(m_nec,0.0);
  if(COLL_CHECK==false)
  {
    return ret;
  }
 
  vector<double> q(robot_kdl_.dof);
  for(int j=0; j<robot_kdl_.dof;++j)
  {
    q[j]=x[robot_kdl_.dof+j];
  }

 
  return  ret;
  
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  vector<double> retval(m_dim,0.0);
  vector<double> q;
  vector<double> ee_pose_arr;

  Eigen::VectorXd f_pose_err;
  Eigen::VectorXd finger_gradient;
  vector<double> gradient;
  gradient.resize(robot_kdl_.dof,0.0);

  //obj_pose_err.resize(6);
  q.resize(robot_kdl_.dof,0.0);
  int finger=int(data_arr_[1][0]);// rigid_finger
  


  
  //int i=timesteps_-1;
  for(int i=0;i<timesteps_;++i)
  {
    
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }


    
    ee_pose_arr=robot_kdl_.getFK(finger,q,true);
    
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],3);// only position
    f_pose_err=des_poses[0]-ee_pose;
    Eigen::MatrixXd link_J;
    robot_kdl_.getPseudoInverse(robot_kdl_.getJacobian(finger,q).block(0,0,3,4),link_J,0.0001);

    if(i==timesteps_-1)
    {
      finger_gradient=-1.0*link_J*f_pose_err;
      for (int k=0;k<4;++k)
      {
        gradient[k]+=finger_gradient[k];
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

vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nic,0.0);
  if(COLL_CHECK==false)
  {
    return retval;
  }
  
  // Add collision constraint between links and object:

  int finger=int(data_arr_[1][0]);// rigid_finger
  vector<double> q(robot_kdl_.dof);
  for(int j=0; j<robot_kdl_.dof;++j)
  {
    q[j]=x[robot_kdl_.dof+j];
  }

  //cerr<<"computing gradients"<<endl;
  // Contact constraint for fingertip:
  vector<double> ee_pose_arr;
  vector<int> link_idx={1,2,4,5,7,8,10,11};
  vector<int> f_idx;//(2);
  vector<int> joint_idx_low;//={0,0,4,4,8,8,12,12};
  vector<int> joint_idx_up;//={2,3,6,7,10,11,15,16};
  vector<int> kdl_link_idx;//={0,1,3,4,6,7,9,10};
  switch(finger){
    case 0:
      f_idx={1,2};
      joint_idx_low={0,0};
      joint_idx_up={2,3};
      kdl_link_idx={0,1};
      break;
    case 1:
      f_idx={4,5};
      joint_idx_low={0,0};
      joint_idx_up={2,3};
      kdl_link_idx={3,4};
      break;
    case 2:
      f_idx={7,8};
      joint_idx_low={0,0};
      joint_idx_up={2,3};
      kdl_link_idx={6,7};
      break;
    case 3:
      f_idx={10,11};
      joint_idx_low={0,0};
      joint_idx_up={3,4};
      kdl_link_idx={9,10};
      break;
  }
  vector<vector<double>> r_poses;
  for (int i=0;i<2;++i)
  {
    vector<double> f_q(&q[joint_idx_low[i]],&q[joint_idx_up[i]]);
    ee_pose_arr=robot_kdl_.getLinkPose(f_q,kdl_link_idx[i]);//robot_kdl_.getFK(finger,q,false);
    r_poses.push_back(ee_pose_arr);
  }
  
  vector<vector<collision_data>> c_data=c_checker.check_collisions(r_poses,f_idx);
  Eigen::VectorXd finger_gradient(4);
  finger_gradient.setZero();

  for (int j=0;j<c_data.size();++j)
  {
    // link jacobian:
    vector<double> f_q(&q[joint_idx_low[j]],&q[joint_idx_up[j]]);
    Map<Eigen::VectorXd> link_pose(&r_poses[j][0],3);
    Eigen::MatrixXd link_J;
    robot_kdl_.getPseudoInverse(robot_kdl_.getLinkJacobian(kdl_link_idx[j],f_q).block(0,0,3,f_q.size()), link_J,0.0001);
    double min_dist=std::abs(c_data[j][0].distance);
    int idx=0;
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

        // Add to finger gradient:
        finger_gradient.segment(0,f_q.size())+=-1.0*link_J*delta_p;
      }
    }
  }
  for(int k=0;k<robot_kdl_.dof;++k)
  {
    retval[4*m_dim+robot_kdl_.dof+k]=finger_gradient[k];
  }

  // Add collision constraint between fingers:
  // self collisions:
  vector<int> coll_idx;
  switch(finger){
    case 0:
      f_idx={1,2,3};
      joint_idx_low={0,0,0};
      joint_idx_up={2,3,4};
      kdl_link_idx={0,1,2};
      coll_idx={4,5,6,7,8,9,10,11,12};
      break;
    case 1:
      f_idx={4,5,6};
      joint_idx_low={0,0,0};
      joint_idx_up={2,3,4};
      kdl_link_idx={3,4,5};
      coll_idx={1,2,3,7,8,9,10,11,12};
      break;
    case 2:
      f_idx={7,8,9};
      joint_idx_low={0,0,0};
      joint_idx_up={2,3,4};
      kdl_link_idx={6,7,8};
      coll_idx={1,2,3,4,5,6,10,11,12};
      break;
    case 3:
      f_idx={10,11,12};
      joint_idx_low={0,0,0};
      joint_idx_up={3,4,4};
      kdl_link_idx={9,10,11};
      coll_idx={1,2,3,4,5,6,7,8,9};
      break;
  }

  finger_gradient.setZero();
  r_poses.clear();
  
  for (int i=0;i<3;++i)
  {
    vector<double> f_q(&q[joint_idx_low[i]],&q[joint_idx_up[i]]);
    ee_pose_arr=robot_kdl_.getLinkPose(f_q,kdl_link_idx[i]);//robot_kdl_.getFK(finger,q,false);
    r_poses.push_back(ee_pose_arr);
  }
  c_data=c_checker.check_self_collisions(r_poses,f_idx,coll_idx);

  
  for (int j=0;j<c_data.size();++j)
  {
    vector<double> f_q(&q[joint_idx_low[j]],&q[joint_idx_up[j]]);
    Map<Eigen::VectorXd> link_pose(&r_poses[j][0],3);
    Eigen::MatrixXd link_J;
    robot_kdl_.getPseudoInverse(robot_kdl_.getLinkJacobian(kdl_link_idx[j],f_q).block(0,0,3,f_q.size()), link_J,0.0001);
  
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
        // Add to finger gradient:
        finger_gradient.segment(0,f_q.size())+=-1.0*link_J*delta_p;
      }
    }
  }
  
  for(int k=0;k<robot_kdl_.dof;++k)
  {
    retval[5*m_dim+robot_kdl_.dof+k]=finger_gradient[k];
  }
  // contact gradient:
  
  //int finger=int(data_arr_[1][0]);// finger to gait
  finger_gradient.setZero();
  r_poses.clear();

  link_idx={3,6,9,12};
  f_idx={link_idx[finger]};
  Eigen::MatrixXd link_J;
  robot_kdl_.getPseudoInverse(robot_kdl_.getJacobian(finger,q).block(0,0,3,4),link_J,0.0001);

  ee_pose_arr=robot_kdl_.getFK(finger,q,false);
  Map<Eigen::VectorXd>  link_pose(&ee_pose_arr[0],3);
  r_poses.push_back(ee_pose_arr);
  c_data=c_checker.check_collisions(r_poses,f_idx);
  for (int j=0;j<c_data.size();++j)
  {
    double min_dist=std::abs(c_data[j][0].distance);
    int idx=0;

    for (int i=0;i<c_data[j].size();++i)
    {
      if(!(c_data[j][i].distance>data_arr_[4][0] && c_data[j][i].distance<data_arr_[4][1]))
      {
        //cerr<<c_data[j][i].distance<<endl;
        if(std::abs(c_data[j][i].distance)<min_dist)
        {
          min_dist=std::abs(c_data[j][i].distance);
          idx=i;
        }
      }
    }
    if(!(c_data[j][idx].distance>data_arr_[4][0] && c_data[j][idx].distance<data_arr_[4][1]))
    {
        // Get distance to move
      Map<Eigen::VectorXd> point(&c_data[j][idx].point[0],3);
      Map<Eigen::VectorXd> dir(&c_data[j][idx].dir[0],3);
      double p_d;
      p_d=c_data[j][idx].distance;
      Eigen::VectorXd delta_p;
      delta_p=link_pose-(point-p_d*dir);
      finger_gradient=-1.0*link_J*delta_p;
    }

  }
  for(int k=0;k<robot_kdl_.dof;++k)
  {
    retval[(4+2)*m_dim+robot_kdl_.dof+k]=finger_gradient[k]*10.0;

  }

  
  return retval;
}

vector<double> optProblem::eqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nec,0.0);
  if(COLL_CHECK==false)
  {
    return retval;
  }

  // contact gradient:
  vector<double> q(robot_kdl_.dof);
  for(int j=0; j<robot_kdl_.dof;++j)
  {
    q[j]=x[robot_kdl_.dof+j];
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
  //vector<double> eq_grad=eqGradient(x);
  vector<double> ineq_grad=inEqGradient(x);
  
  grad.resize(m_dim,0.0);

  for(int i=0;i<m_dim;i++)
  {
    grad[i]=obj_grad[i];
  }


  /*

  for(int i=m_dim+2*robot_kdl_.dof;i<m_dim+2*robot_kdl_.dof+robot_kdl_.dof*(timesteps_-1);i++)
  {
      grad.emplace_back(ineq_grad[i-m_dim-robot_kdl_.dof*2]);
      grad.emplace_back(ineq_grad[i-m_dim-robot_kdl_.dof]);
  }
  */
  
  grad.emplace_back(ineq_grad[robot_kdl_.dof*3+0]);
  grad.emplace_back(ineq_grad[robot_kdl_.dof*3+1]);
  grad.emplace_back(ineq_grad[robot_kdl_.dof*3+2]);
  grad.emplace_back(ineq_grad[robot_kdl_.dof*3+3]);
  
  grad.emplace_back(ineq_grad[robot_kdl_.dof*5+0]);
  grad.emplace_back(ineq_grad[robot_kdl_.dof*5+1]);
  grad.emplace_back(ineq_grad[robot_kdl_.dof*5+2]);
  grad.emplace_back(ineq_grad[robot_kdl_.dof*5+3]);

  for(int i=0;i<robot_kdl_.dof;++i)
  {
    grad.emplace_back(ineq_grad[6*m_dim+robot_kdl_.dof+i]);
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


  
  // inequality constraints:
  /*
  for (int i=0;i<robot_kdl_.dof*(timesteps_-1);++i)
  {
    retval.emplace_back(i+m_nec+1,i);
    retval.emplace_back(i+m_nec+1,i+robot_kdl_.dof);
    
  }
  */
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1), 4);
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1), 5);
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1), 6);
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1), 7);

  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1)+1, 4);
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1)+1, 5);
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1)+1, 6);
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1)+1, 7);

  
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1)+2, 4);
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1)+2, 5);
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1)+2, 6);
  retval.emplace_back(1+m_nec+robot_kdl_.dof*(timesteps_-1)+2, 7);

  return retval;
}
vector<vector<double>> optProblem::bounds() const
{
  vector<vector<double>> bound;
  vector<double> up_bounds;
  vector<double> low_bounds;
  for(int i=0;i<timesteps_;++i)
  {
    
    up_bounds.insert(up_bounds.end(), robot_kdl_.up_bounds.begin()+(int(data_arr_[1][0])*4), robot_kdl_.up_bounds.begin()+int(data_arr_[1][0])*4+4);
    low_bounds.insert(low_bounds.end(), robot_kdl_.low_bounds.begin()+int(data_arr_[1][0])*4, robot_kdl_.low_bounds.begin()+int(data_arr_[1][0])*4+4);
  }
  bound.resize(2);
  bound[0]=low_bounds;
  bound[1]=up_bounds;
  return bound;
  
}


