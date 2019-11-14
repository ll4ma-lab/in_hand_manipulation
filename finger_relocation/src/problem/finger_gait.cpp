#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT true
#endif
#include "finger_relocation/problem/hand_problem.hpp"
using namespace opt_problems;

optProblem::optProblem()
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

bool optProblem::updateScene(in_hand_manipulation::handEnv &hand_env)
{
  hand_env_ = &hand_env;
  robot_kdl_ = &hand_env.robot_kdl_;
  hand_dof_ = hand_env.hand_dof_;
  return true;
}


bool optProblem::updateData(const Eigen::VectorXd &des_pose, const vector<double> &vel_limits, const int &finger)
{  
  m_dim=hand_dof_*timesteps_;
  m_nec=0;
  m_nic=hand_dof_*(timesteps_);//+timesteps_+1+timesteps_+1;
  // velocity constraints, link collision(t_steps), f_tip contact at T, link collisions, stability constraint
  des_pose_=des_pose;
  
  vel_lim=vel_limits[0];
  finger_=finger;
  
  l_idx_={finger_*3,finger_*3+1,finger_*3+2};
  f_tip_=finger_*3+2;
  
  dist_thresh_=0.0001;// 5mm
  stability_thresh_=10.005;// 1cm
  // get initial pose:
  Map<Eigen::VectorXd> j0(&joints_0[0],joints_0.size());
  init_joints=j0;
  //cerr<<j0.transpose()<<endl;
  //robot_kdl_->getFK(finger_,j0,initial_pose_,true);
}

double optProblem::objFunction(vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;

  
  // get robot state:
  hand_env_->update_robot_state(x);
  Eigen::Map<Eigen::VectorXd> final_q(&x[(timesteps_-1)*hand_dof_],hand_dof_);
  Eigen::VectorXd reach_pose;
  robot_kdl_->getFK(finger_,final_q,reach_pose,true);

  // compute pose error:
  
  cost_val = (des_pose_.head(3) - reach_pose.head(3)).squaredNorm();
  return cost_val;
  /*
  //return cost_val;
  double dist_err=0.0;
  Eigen::VectorXd q;
  Eigen::VectorXd pos_err;
 
  q.resize(robot_kdl_->dof);
  
  Eigen::VectorXd ee_pose,obj_pose_err;
  Eigen::VectorXd acc_cost(robot_kdl_->dof);

  Eigen::VectorXd finger_q;
  finger_q.resize(4);
  Eigen::VectorXd l_pose_;
  vector<vector<double>> link_poses;
  link_poses.resize(3*4);

  // Minimum acceleration cost:
  vector<Eigen::VectorXd> q_arr;
  Eigen::VectorXd j0(joints_0.size());
  for(int i=0;i<joints_0.size();++i)
  {
    j0[i]=joints_0[i];
  }
  //Map<Eigen::VectorXd> j0(&joints_0[0], joints_0.size());
  
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


  Eigen::VectorXd ftip_normal(4);
  ftip_normal.setZero();
  ftip_normal[2]=-1.0;
  ftip_normal[3]=1.0;
  for(int i=0; i<timesteps_;++i)
  {
    if(timesteps_>2)
    {
      acc_cost=q_arr[i-2+2]-2.0*q_arr[i+2-1]+q_arr[i+2];
      cost_val+=0.01*acc_cost.squaredNorm();
    }
    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }

    if(i==timesteps_-1)
    {
      if(des_pose_.size()!=4)
      {
        robot_kdl_->getFK(finger_,q.segment(finger_*4,4),ee_pose,false);
        obj_pose_err=des_pose_.head(3)-ee_pose.head(3);
      }
      else
      {
        obj_pose_err=des_pose_.head(4)-q.segment(finger_*4,4);
      }
      cost_val+=100.0*obj_pose_err.squaredNorm();
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
  //cerr<<"cost:"<<cost_val<<endl;
  return cost_val;
  */
}

vector<double> optProblem::objGradient(vector<double> x) const
{

  vector<double> retval(m_dim,0.0);
  
  // get robot state:
  hand_env_->update_robot_state(x);
  Eigen::Map<const Eigen::VectorXd> final_q(&x[(timesteps_-1)*hand_dof_],hand_dof_);
  Eigen::VectorXd reach_pose;
  robot_kdl_->getFK(finger_,final_q,reach_pose,true);

  // compute pose error:
  Eigen::VectorXd err_vec = des_pose_.head(3) - reach_pose.head(3);

  // get jacobian
  Eigen::MatrixXd point_J;
  robot_kdl_->getJacobian(finger_,final_q,point_J);

  Eigen::VectorXd gradient = -2.0 * point_J.transpose() * err_vec;

  //cerr<<gradient.size()<<endl;
  for(int i =0;i<hand_dof_;++i)
  {
    retval[i] = gradient[hand_env_->arm_dof_+i];
  }

  //robot_kdl_->getJacobian
  /*
  //return retval;
  Eigen::VectorXd q;
  Eigen::VectorXd acc_cost,acc_grad;
  Eigen::VectorXd finger_gradient(4);
  finger_gradient.setZero();
  vector<double> gradient;
  gradient.resize(robot_kdl_->dof,0.0);

  q.resize(robot_kdl_->dof);
  
  Eigen::VectorXd ee_pose,obj_pose_err;
  Eigen::MatrixXd J;

    vector<Eigen::VectorXd> q_arr;
  Eigen::VectorXd j0(joints_0.size());
  for(int i=0;i<joints_0.size();++i)
  {
    j0[i]=joints_0[i];
  }
  //Map<Eigen::VectorXd> j0(&joints_0[0], joints_0.size());
  Eigen::VectorXd ftip_normal(4);
  ftip_normal.setZero();
  ftip_normal[2]=-1.0;
  ftip_normal[3]=1.0;
  
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
    
    q=q_arr[i+2];
    //gradient.setZero();
    // min acc grad:
    if(timesteps_>2)
    {
      acc_cost=0.01*(q_arr[i-2+2]-2.0*q_arr[i+2-1]+q_arr[i+2]);
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
    }

    
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }


    if(i==timesteps_-1)
    {
      if(des_pose_.size()!=4)
      {
        robot_kdl_->getFK(finger_,q.segment(finger_*4,4),ee_pose,false);
        obj_pose_err=des_pose_.head(3)-ee_pose.head(3);

        robot_kdl_->getJacobian(finger_,q.segment(finger_*4,4),J);
        finger_gradient=-2.0*100.0*obj_pose_err.transpose()*J.block(0,0,3,J.cols());
      }
      else
      {
        obj_pose_err=des_pose_.head(4)-q.segment(finger_*4,4);
        finger_gradient=2.0*(obj_pose_err);
      }
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
  */
  return retval;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
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
  */
  // limiting velocity at joints
  vector<double> vel_const(m_nic,0.0);
  /*
  Eigen::VectorXd hand_q(16),obj_pose_err;
 
  for(int j=finger_*4;j<finger_*4+4;++j)
  {
    vel_const[(0)*robot_kdl_->dof+j]=std::abs(x[(0)*robot_kdl_->dof+j]-joints_0[j])-vel_lim;
  }

  for (int i=1; i<timesteps_-1; ++i)
  {
    for(int j=finger_*4;j<finger_*4+4;++j)
    {
      vel_const[(i)*robot_kdl_->dof+j]=std::abs(x[(i)*robot_kdl_->dof+j]-x[(i-1)*robot_kdl_->dof+j])-vel_lim;
    }
  }

  //return vel_const;
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
      //finger joints:
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
        robot_kdl_->getLinkPose(j*3+i,finger_q,l_pose_);
        robot_kdl_->getSTDVector(l_pose_,link_poses[j*3+i]);
      }
    }
    
    // get contact data:
    vector<coll_data> contact_data;
    //c_check->check_contacts(link_poses,contact_data,l_idx_);
    collision_err=0.0;

    int j=finger_;

    for(int i=0;i<3;++i)
    {
      
      // For all timesteps except first and last
      if(t<timesteps_-2 && i<3 && t>2)
      {
        double s_d=contact_data[j*3+i].distance;
        if(s_d<dist_thresh_)
        {
          collision_err+=std::abs(s_d);
        }
        
      }
      
      if(t==(timesteps_-1) && i<2)
      {
        double s_d=contact_data[j*3+i].distance;
        if(s_d<dist_thresh_)
        {
          collision_err+=std::abs(s_d);
        } 
      }
      if(t==(timesteps_-1) && i==2)
      {
        double s_d=contact_data[j*3+i].distance;
        if(s_d>0.0)
        {
          collision_err+=std::abs(s_d);
        } 

      }
    }
    
    vel_const[robot_kdl_->dof*timesteps_+t]=collision_err;
    
    // check link to link collsions:
    contact_data.clear();
  }
  
  Eigen::VectorXd q(16),ee_pose;//obj_pose_err;
  // stability cost:
  for(int j=0; j<robot_kdl_->dof;++j)
  {
    q[j]=x[(timesteps_-1)*robot_kdl_->dof+j];
  }

  robot_kdl_->getFK(finger_,q.segment(finger_*4,4),ee_pose,true);
  obj_pose_err=initial_pose_.head(3)-ee_pose.head(3);
  //vel_const[robot_kdl_->dof*timesteps_+timesteps_*2+1]=obj_pose_err.squaredNorm()-stability_thresh_;

  for(int i=0;i<vel_const.size();++i)
  {
    if(vel_const[i]>0.0)
    {
      //cerr<<i<<endl;
    }
  }
  */
  return vel_const;
}


vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(1*robot_kdl_->dof+2*(timesteps_-1)*robot_kdl_->dof,0.0);
  /*
  //return retval;
  for(int j=finger_*4;j<finger_*4+4;++j)
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
      for(int j=finger_*4;j<finger_*4+4;++j)
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
  */
  return retval;

}
vector<double> optProblem::collisionGradientFD(vector<double> x) const
{
  vector<double> retval(robot_kdl_->dof*(timesteps_+2+timesteps_),0.0);
  return retval;  
}

vector<double> optProblem::collisionGradient(vector<double> x) const
{
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
  
  vector<double> retval(robot_kdl_->dof*(timesteps_+2+timesteps_),0.0);
  /*
  //return retval;
  Eigen::VectorXd hand_q(16),obj_pose_err;
  //double collision_err=0.0;
  Eigen::VectorXd gradient,finger_gradient,contact_gradient;
  finger_gradient.resize(4);
  contact_gradient.resize(4);
  gradient.resize(16);
  gradient.setZero();
  vector<Eigen::MatrixXd> hand_J(12);
  //for(int t=0;t<timesteps_;++t)
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
    finger_q.tail(4)=hand_q.segment(4*finger_,4);
    
    // get contact data:
    vector<coll_data> contact_data;
    //c_check->check_contacts(link_poses,contact_data,l_idx_);
    
    int j=finger_;

    for(int i=0;i<3;++i)
    {
      int n_js=hand_J[j*3+i].cols();

      
      if(t<timesteps_-2 && t>2 && i<3)
      {
        Map<Eigen::VectorXd> link_pose(&link_poses[j*3+i][0],3);
        
        double s_d=contact_data[j*3+i].distance;
        Eigen::VectorXd delta_p=contact_data[j*3+i].p2-contact_data[j*3+i].p1;
          
        if(s_d<dist_thresh_)
        {
          Eigen::MatrixXd j_mat;
          j_mat=compute_jacobian(j*3+i,finger_q,contact_data[j*3+i].p1);
          finger_gradient.segment(0,n_js)+=1.0*copysign(1.0, s_d)*(j_mat*delta_p).segment(0,n_js);

        }
        
        gradient.segment(j*4,4)+=finger_gradient;
        finger_gradient.setZero();
      }
      
      if(t==timesteps_-1 && i<2)
      {
        Map<Eigen::VectorXd> link_pose(&link_poses[j*3+i][0],3);
        double s_d=contact_data[j*3+i].distance;
        Eigen::VectorXd delta_p=contact_data[j*3+i].p2-contact_data[j*3+i].p1;

        if(s_d<dist_thresh_)
        {
          
          Eigen::MatrixXd j_mat;
          j_mat=compute_jacobian(j*3+i,finger_q,contact_data[j*3+i].p1);
          finger_gradient.segment(0,n_js)+=1.0*copysign(1.0, s_d)*(j_mat*delta_p).segment(0,n_js);
        }
        
        gradient.segment(j*4,4)+=finger_gradient;
        finger_gradient.setZero();
        
      }
      
      if(t==timesteps_-1 && i==2)
      {
        Map<Eigen::VectorXd> link_pose(&link_poses[j*3+i][0],3);
        double s_d=contact_data[j*3+i].distance;
        Eigen::VectorXd delta_p=contact_data[j*3+i].p2-contact_data[j*3+i].p1;
          
        if(s_d>0.0)
        {
          Eigen::MatrixXd j_mat;
          j_mat=compute_jacobian(j*3+i,finger_q,contact_data[j*3+i].p1);
          finger_gradient.segment(0,n_js)+=-1.0*copysign(1.0, s_d)*(j_mat*delta_p).segment(0,n_js);
        }
        
        gradient.segment(j*4,4)+=finger_gradient;
        
      }
      
    }
    
    // add robot collision gradient to main
    for(int k=finger_*4;k<finger_*4+4;++k)
    {
      retval[t*robot_kdl_->dof+k]+=gradient[k];
    }
        
    gradient.setZero();
    
  }
  // stability gradient:
  Eigen::VectorXd ee_pose,q(robot_kdl_->dof);
  for(int j=0; j<robot_kdl_->dof;++j)
  {
    q[j]=x[(timesteps_-1)*robot_kdl_->dof+j];
  }

  robot_kdl_->getFK(finger_,q.segment(finger_*4,4),ee_pose,true);
  // object pose error:
  obj_pose_err=initial_pose_.head(3)-ee_pose.head(3);
  Eigen::MatrixXd J;
  robot_kdl_->getJacobian(finger_,q.segment(finger_*4,4),J);
  finger_gradient.setZero();
  finger_gradient=-2.0*obj_pose_err.transpose()*J.block(0,0,3,J.cols());
  gradient.setZero();
  gradient.segment(finger_*4,4)=finger_gradient;
  for(int k=finger_*4;k<finger_*4+4;++k)
  {
    //retval[timesteps_*robot_kdl_->dof+robot_kdl_->dof+timesteps_*robot_kdl_->dof+k]+=gradient[k];
  }    
  */  
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
    if(robot_kdl_->up_bounds.size()>16)
    {
    up_bounds.insert(up_bounds.end(), robot_kdl_->up_bounds.begin()+7, robot_kdl_->up_bounds.end());
    low_bounds.insert(low_bounds.end(), robot_kdl_->low_bounds.begin()+7, robot_kdl_->low_bounds.end());
    }
    else
    {
      up_bounds.insert(up_bounds.end(), robot_kdl_->up_bounds.begin(), robot_kdl_->up_bounds.end());
      low_bounds.insert(low_bounds.end(), robot_kdl_->low_bounds.begin(), robot_kdl_->low_bounds.end());
    }
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
    for(int j=finger_*4;j<finger_*4+4;++j)
    {
      grad.emplace_back(collision_grad[i*robot_kdl_->dof+j]);
    }
    //cerr<<i<<" "<<collision_grad[i*robot_kdl_->dof]<<endl;
  }

  
  // contact gradient:
  for(int j=finger_*4;j<finger_*4+4;++j)
  {
    grad.emplace_back(collision_grad[timesteps_*robot_kdl_->dof+j]);
    //cerr<<collision_grad[timesteps_*robot_kdl_->dof+j]<<endl;
  }

  // link collision gradient:
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=finger_*4;j<finger_*4+4;++j)
    {
      grad.emplace_back(collision_grad[timesteps_*robot_kdl_->dof+robot_kdl_->dof+i*robot_kdl_->dof+j]);
    }
    //cerr<<i<<" "<<collision_grad[i*robot_kdl_->dof]<<endl;
  }

  // stability gradient:
  for(int j=0;j<robot_kdl_->dof;++j)
  {
    grad.emplace_back(collision_grad[2*timesteps_*robot_kdl_->dof+robot_kdl_->dof+j]);
  }

  //cerr<<"grad size: "<<grad.size()<<endl;

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
  for(int i=0;i<hand_dof_;++i)
  {
    retval.emplace_back(1+m_nec+i,i);
  }
  // Other timesteps:
  //for(int i=0;
  for(int i=1;i<timesteps_;++i)
  {
    for(int j=0;j<hand_dof_;++j)
    {
      retval.emplace_back(1+m_nec+1*hand_dof_+(i-1)*hand_dof_+j, (i-1)*hand_dof_+j);
      retval.emplace_back(1+m_nec+1*hand_dof_+(i-1)*hand_dof_+j,(i)*hand_dof_+j);
    }
  }

  // object collision gradient
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=finger_*4;j<finger_*4+4;++j)
    {
      retval.emplace_back(1+m_nec+hand_dof_*timesteps_+i,i*hand_dof_+j);
    }
  }

  // contact gradient
  for(int j=finger_*4;j<finger_*4+4;++j)
  {
    retval.emplace_back(1+m_nec+hand_dof_*timesteps_+timesteps_,(timesteps_-1)*hand_dof_+j);
  }

  // link collision gradient
  for(int i=0;i<timesteps_;++i)
  {
    for(int j=finger_*4;j<finger_*4+4;++j)
    {
      retval.emplace_back(1+m_nec+hand_dof_*timesteps_+timesteps_+1+i,i*hand_dof_+j);
    }
  }
  // stability constraint:
  
  for(int j=0;j<hand_dof_;++j)
  {
    retval.emplace_back(1+m_nec+hand_dof_*timesteps_+(timesteps_*2)+1,(timesteps_-1)*hand_dof_+j);
  }
  for(int i=0;i<retval.size();++i)
  {
    //cerr<<get<0>(retval[i])<<" "<<get<1>(retval[i])<<endl;
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

