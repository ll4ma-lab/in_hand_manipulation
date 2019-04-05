#define SPARSE_GRADIENT true
#include <in_grasp_manipulation/problems/in_grasp_opt_problem.hpp>
//#define COLL_CHECK false
namespace opt_problems
{
bool optProblem::updateData( vector<double> vel_limits,vector<vector<double>> data_arr, vector<Eigen::MatrixXd> data_mats)
{
  m_dim=robot_kdl_->dof*timesteps_;
  m_nec=0;//robot_kdl.dof;
  m_nic=robot_kdl_->dof*(timesteps_)+(timesteps_*2)*data_arr[3].size();// velocity constraints, rr constraints

  // if collision checking is enabled, add collision constraints:
  if(collision_checking_)
  {
    m_nic+=timesteps_; // we check if object is in collision with the hand at every timestep
  }
  
  des_poses.resize(timesteps_);
  vel_lim=vel_limits[0];
  data_arr_=data_arr;
  data_mats_=data_mats;
  //Eigen::VectorXd des_pose(data_arr[0].data());
  Eigen::Map<Eigen::VectorXd> des_pose(&data_arr[0][0],data_arr[0].size());// desired object pose
  
  Eigen::Map<Eigen::VectorXd> initial_pose( &data_arr[4+data_arr[3].size()][0],
                                            data_arr[4+data_arr[3].size()].size());// desired object pose
  for(int i=0;i<timesteps_;++i)
  {
    des_poses[i]=initial_pose+(des_pose-initial_pose)*(float(i+1)/float(timesteps_));
  }


  return true;
}
double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;
  Eigen::VectorXd q;
  Eigen::VectorXd obj_pos_err;
  int rigid_f=int(data_arr_[2][0]);// rigid_finger

  double wo,wr,wd,wi;
  wo=data_arr_[1][0];
  wr=data_arr_[1][1];
  wd=data_arr_[1][2];
  wi=data_arr_[1][3];
  q.resize(robot_kdl_->dof);

  vector<double> o_err;

  double tot_pos_cost=0.0;
  double debug_cost=0.0;
  vector<Eigen::VectorXd> j_poses;
  // store finger pose for min acceleration cost:
  vector<double> init_joints=joints_0;


  Eigen::Map<Eigen::VectorXd>j_0(&init_joints[0],init_joints.size());

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
  j_poses.push_back(q);
  j_poses.push_back(q);

  
  Eigen::VectorXd acc_cost(16),ee_pose;
  

  Eigen::MatrixXd f_mat,thumb_mat;

  
  for(int i=0; i<timesteps_;++i)
  {
  
    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }


    if(i==timesteps_-1)
    {
      robot_kdl_->getFK(rigid_f,q.segment(rigid_f*4,4),ee_pose,true);
      
      vector<double> o_1(ee_pose.data()+3, ee_pose.data()+6);
      vector<double> o_d(des_poses[i].data()+3, des_poses[i].data()+6);
      
      // object pose error:
      
      obj_pos_err=des_poses[i]-ee_pose;
      robot_kdl_->euler_diff(o_1,o_d,o_err);
      obj_pos_err[3]=o_err[0]*0.1;
      obj_pos_err[4]=o_err[1]*0.1;
      obj_pos_err[5]=o_err[2]*0.1;
      cost_val+=wd*obj_pos_err.squaredNorm();
      
    }

    // min_acceleration cost:
    if(timesteps_>1)
    {
      acc_cost=(j_poses[i-2+2]-2.0*j_poses[i+2-1]+j_poses[i+2]);
      cost_val+=wi*acc_cost.squaredNorm();
      //debug_cost+=wi*acc_cost.squaredNorm();
    }


  }
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
  Eigen::VectorXd q(robot_kdl_->dof);
  vector<double> ee_pose_arr;

  Eigen::VectorXd obj_pose_err;
  Eigen::VectorXd finger_gradient;
  vector<double> gradient;
  gradient.resize(robot_kdl_->dof,0.0);

  obj_pose_err.resize(6);
  
  int rigid_f=int(data_arr_[2][0]);// rigid_finger
  

  vector<double> init_joints=joints_0;
  vector<double> o_err;
  Eigen::Map<Eigen::VectorXd>j_0(&init_joints[0],init_joints.size());
  

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
  j_poses.push_back(q);
  j_poses.push_back(q);

  Eigen::VectorXd ee_pose;
  Eigen::MatrixXd thumb_mat,f_mat,J;
  for(int i=0;i<timesteps_;++i)
  {
    
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }

    if(i==timesteps_-1)
    {
      robot_kdl_->getFK(rigid_f,q.segment(rigid_f*4,4),ee_pose,true);
      
      // compute error using kdl:
      vector<double> o_1(ee_pose.data()+3, ee_pose.data()+6);
      vector<double> o_d(des_poses[i].data()+3, des_poses[i].data()+6);
      
      robot_kdl_->euler_diff(o_1,o_d,o_err);
      
      obj_pose_err=des_poses[i]-ee_pose;
    
    
      obj_pose_err[3]=o_err[0]*0.1*0.1;
      obj_pose_err[4]=o_err[1]*0.1*0.1;
      obj_pose_err[5]=o_err[2]*0.1*0.1;
      
      robot_kdl_->getJacobian(rigid_f,q.segment(rigid_f*4,4),J);
      //cerr<<J<<endl;
      finger_gradient=-2.0*wd*J.transpose()*obj_pose_err;

      //cerr<<"AG: ";//<<finger_gradient.transpose()<<endl;

      for (int k=0;k<4;++k) 
      {
        gradient[rigid_f*4+k]+=finger_gradient[k];
        
      }
      //cerr<<endl;

      /*
      vector<double> state(q.data(),q.data()+q.size());
      vector<double> grad_fd=debugGrad(state);

      cerr<<"FD: ";
      for(int k=0;k<4;++k)
      {
        //gradient[rigid_f*4+k]=grad_fd[rigid_f*4+k];
        //cerr<<gradient[rigid_f*4+k]<<" ";
      }
      cerr<<endl;

      */
      for (int j=0;j<robot_kdl_->dof;++j)
      {
        retval[j+robot_kdl_->dof*(i)]+=gradient[j];
      }
      fill(gradient.begin(), gradient.end(), 0.0);

    }
    
    if(timesteps_>1)
    {
      // min_acceleration gradient:
      Eigen::VectorXd acc_cost(16);
      Eigen::VectorXd grad;
      acc_cost=wi*(j_poses[i-2+2]-2.0*j_poses[i-1+2]+j_poses[i+2]);

      grad=2.0*acc_cost;
      for (int j=0;j<16;++j)
      {
        retval[j+robot_kdl_->dof*(i)]+=grad[j];
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
    }


  
    fill(gradient.begin(), gradient.end(), 0.0);
  }

  return retval;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  
  // limiting velocity at joints
  vector<double> vel_const(m_nic,0.0);
  //return vel_const;
  
 
  for(int j=0;j<robot_kdl_->dof;++j)
  {
    double vel=x[(0)*robot_kdl_->dof+j]-joints_0[j];
    vel_const[(0)*robot_kdl_->dof+j]=pow((max(abs(vel),vel_lim)-vel_lim),2);

  }

  for (int i=1; i<timesteps_; ++i)
  {
    for(int j=0;j<robot_kdl_->dof;++j)
    {
      double vel=x[(i)*robot_kdl_->dof+j]-x[(i-1)*robot_kdl_->dof+j];
      vel_const[(i)*robot_kdl_->dof+j]=pow((max(abs(vel),vel_lim)-vel_lim),2);
    }
  }
  // relaxed rigidity constraints:
  Eigen::VectorXd q;
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
    Eigen::Map<Eigen::VectorXd> pos_offset(&p_off[0],p_off.size());
    pos_offsets.emplace_back(pos_offset);
  }
  double wo,wr,wd,wi;
  wo=data_arr_[1][0];
  wr=data_arr_[1][1];
  wd=data_arr_[1][2];
  wi=data_arr_[1][3];
  q.resize(robot_kdl_->dof);

  vector<double> o_err;

  double tot_pos_cost=0.0;
  double debug_cost=0.0;
  vector<Eigen::VectorXd> j_poses;
  // store finger pose for min acceleration cost:
  vector<double> init_joints=joints_0;


  Eigen::Map<Eigen::VectorXd>j_0(&init_joints[0],init_joints.size());

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
  //cerr<<r_f_poses.size()<<endl;
  j_poses.push_back(j_poses[timesteps_+1]);
  j_poses.push_back(j_poses[timesteps_+1]);
   
  Eigen::VectorXd acc_cost(16),ee_pose;
  

  Eigen::MatrixXd f_mat,thumb_mat;
  
  for(int i=0; i<timesteps_;++i)
  {
    double position_cost=0.0;
    double orientation_cost=0.0;

    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }
    robot_kdl_->getFK(rigid_f,q.segment(rigid_f*4,4),ee_pose,true);
    

    // Relaxed-Rigidity costs:
    // get current thumb mat:
    robot_kdl_->getFK(rigid_f,q.segment(rigid_f*4,4),thumb_mat);
    
    for(int k=0;k<data_arr_[3].size();++k)
    {
      int f=data_arr_[3][k];
      if(f!=rigid_f)
      {
        // Position:
        robot_kdl_->getFK(f,q.segment(f*4,4),f_mat);
        Eigen::MatrixXd des_mat=thumb_mat*data_mats_[f];


	// compute difference between the two frames:
        
	
        Eigen::VectorXd pose_diff(6);
        
        pose_diff[0]=des_mat(0,3)-f_mat(0,3);
        pose_diff[1]=des_mat(1,3)-f_mat(1,3);
        pose_diff[2]=des_mat(2,3)-f_mat(2,3);
        
        
        // Compute orientation error:
        robot_kdl_->euler_diff(f_mat,des_mat,o_err);
        
        pos_err=pose_diff.segment(0,3)*1.0;
        
        orient_err[1]=o_err[1];
        orient_err[0]=0.0;
        orient_err[2]=0.0;
        position_cost=wr*pos_err.squaredNorm();//(max(pos_err.squaredNorm(),wr)-wr);
        
        orientation_cost=wo*orient_err.squaredNorm();
        vel_const[robot_kdl_->dof*timesteps_+k*timesteps_*2+i]=position_cost;//pow(position_cost,2);
        vel_const[robot_kdl_->dof*timesteps_+k*timesteps_*2+timesteps_+i]=orientation_cost;//pow(position_cost,2);
      }
      
    }
    
  }

  
  // add collision constraints:
  if(collision_checking_)
  {
    for(int i=0;i<timesteps_;++i)
    {
      // get current state:
      
      // get collision constraint:
      vector<double> coll_const=collisionConstraints(x);
      
    }
  }
  return vel_const;
}

vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(1*robot_kdl_->dof+2*(robot_kdl_->dof)*(timesteps_-1),0.0);
  for(int j=0;j<robot_kdl_->dof;++j)
  {
    double vel=x[j]-joints_0[j];
    double delta=2*(max(abs(vel),vel_lim)-vel_lim);
    retval[j]+=1.0*copysign(delta,vel);
  }
 
  for(int i=1;i<timesteps_;++i)
  {
      for(int j=0;j<robot_kdl_->dof;++j)
      {
        double vel=x[(i)*robot_kdl_->dof+j]-x[(i-1)*robot_kdl_->dof+j];
        double delta=2*(max(abs(vel),vel_lim)-vel_lim);
        retval[1*robot_kdl_->dof+2*(i-1)*robot_kdl_->dof+robot_kdl_->dof+j]+=1.0*copysign(delta,vel);
        retval[1*robot_kdl_->dof+2*(i-1)*robot_kdl_->dof+j]+= -1.0*copysign(delta,vel);
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
  vector<double> rr_grad=rrGradient(x);
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

  // rr_gradient:

  for (int k=0;k<data_arr_[3].size();++k)
  {
    
    for(int i=0;i<timesteps_;++i)
    {
      for(int j=0;j<robot_kdl_->dof;++j)
      {
        grad.emplace_back(rr_grad[2*k*timesteps_*robot_kdl_->dof+robot_kdl_->dof*i+j]);
        
      }
    }
    for(int i=0;i<timesteps_;++i)
    {
      for(int j=0;j<robot_kdl_->dof;++j)
      {
        grad.emplace_back(rr_grad[2*k*timesteps_*robot_kdl_->dof+robot_kdl_->dof*timesteps_+robot_kdl_->dof*i+j]);
      }
    }
  }

  return grad;
}
vector<double> optProblem::rrGradient(vector<double> x) const
{
  vector<double> retval(robot_kdl_->dof*2*timesteps_*data_arr_[3].size(),0.0);
  double wo,wr,wd,wi;
  wo=data_arr_[1][0];
  wr=data_arr_[1][1];
  wd=data_arr_[1][2];
  wi=data_arr_[1][3];
  
  Eigen::VectorXd q(robot_kdl_->dof);
  vector<double> ee_pose_arr;

  Eigen::VectorXd obj_pose_err;
  Eigen::VectorXd finger_gradient,rigid_gradient;
  vector<double> p_gradient;
  vector<double> o_gradient;

  p_gradient.resize(robot_kdl_->dof,0.0);
  o_gradient.resize(robot_kdl_->dof,0.0);

  obj_pose_err.resize(6);
  
  int rigid_f=int(data_arr_[2][0]);// rigid_finger
  

  Eigen::VectorXd pos_err,orient_err;
  pos_err.resize(6);
  orient_err.resize(6);
  vector<double> init_joints=joints_0;
  vector<double> o_err;
  Eigen::Map<Eigen::VectorXd>j_0(&init_joints[0],init_joints.size());
  

  // position offsets:
  vector<Eigen::VectorXd> pos_offsets;
  
  for(int k=0;k<data_arr_[3].size();++k)
  {
    vector<double> p_off=data_arr_[4+k];
    Eigen::Map<Eigen::VectorXd> pos_offset(&p_off[0],p_off.size());
    pos_offsets.emplace_back(pos_offset);
  }

  Eigen::VectorXd ee_pose;
  Eigen::MatrixXd thumb_mat,f_mat,J,rigid_J;
  double pos_cost,or_cost;
  
  for(int i=0;i<timesteps_;++i)
  {
    
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }
    
    
     // Relaxed-Rigidity costs:
    robot_kdl_->getFK(rigid_f,q.segment(rigid_f*4,4),thumb_mat);
    Eigen::MatrixXd inv_thumb_mat=thumb_mat.inverse();
    robot_kdl_->getJacobian(rigid_f,q.segment(rigid_f*4,4),rigid_J);

    pos_cost=0.0;
    or_cost=0.0;
    for(int k=0;k<data_arr_[3].size();++k)
    {
      int f=data_arr_[3][k];
      if(f!=rigid_f)
      {
        //Position
        robot_kdl_->getFK(f,q.segment(f*4,4),f_mat);

        Eigen::MatrixXd des_mat=thumb_mat*data_mats_[f];


	// compute difference between these two frames:
	// off_mat and data_mat
        Eigen::VectorXd pose_diff(6);
        pose_diff[0]=des_mat(0,3)-f_mat(0,3);
        pose_diff[1]=des_mat(1,3)-f_mat(1,3);
        pose_diff[2]=des_mat(2,3)-f_mat(2,3);

        robot_kdl_->euler_diff(f_mat,des_mat,o_err);
        
        pos_err=pose_diff;
        pos_err[0]=pos_err[0]*1.0;
        pos_err[1]=pos_err[1]*1.0;
        pos_err[2]=pos_err[2]*1.0;
        pos_err[3]=0.0;
        pos_err[4]=o_err[1];
        pos_err[5]=0.0;
        pos_cost+=pos_err.head(3).squaredNorm();
        
        robot_kdl_->getJacobian(f,q.segment(f*4,4),J);


        or_cost+=pos_err.tail(3).squaredNorm();
        Eigen::VectorXd p_grad=-2.0*pos_err.head(3).transpose()*J.block(0,0,3,J.cols());
        Eigen::VectorXd o_grad=-2.0*pos_err.tail(3).transpose()*J.block(3,0,3,J.cols());
                
        for (int l=0;l<4;++l)
        {
          p_gradient[f*4+l]+=p_grad[l];
          o_gradient[f*4+l]+=o_grad[l];
        }
        //p_grad=2.0*pos_err.head(3).transpose()*rigid_J.block(0,0,3,J.cols());
        //o_grad=2.0*pos_err.tail(3).transpose()*rigid_J.block(3,0,3,J.cols());

        
        // compute gradient for rigid finger:

        des_mat=f_mat*(data_mats_[f].inverse());


	// compute difference between these two frames:
	// off_mat and data_mat
        pose_diff[0]=des_mat(0,3)-thumb_mat(0,3);
        pose_diff[1]=des_mat(1,3)-thumb_mat(1,3);
        pose_diff[2]=des_mat(2,3)-thumb_mat(2,3);

        robot_kdl_->euler_diff(thumb_mat,des_mat,o_err);
        
        pos_err=pose_diff;
        pos_err[0]=pos_err[0]*1.0;
        pos_err[1]=pos_err[1]*1.0;
        pos_err[2]=pos_err[2]*1.0;
        pos_err[3]=0.0;
        pos_err[4]=o_err[1];
        pos_err[5]=0.0;
                
        // rigid finger gradient:
        p_grad=-2.0*pos_err.head(3).transpose()*rigid_J.block(0,0,3,J.cols());
        o_grad=-2.0*pos_err.tail(3).transpose()*rigid_J.block(3,0,3,J.cols());
        for (int l=0;l<4;++l)
        {
          p_gradient[rigid_f*4+l]+=p_grad[l];
          o_gradient[rigid_f*4+l]+=o_grad[l];
        }
        for(int l=0;l<robot_kdl_->dof;++l)
        {
          retval[2*k*timesteps_*robot_kdl_->dof+robot_kdl_->dof*(i)+l]+=wr*p_gradient[l];
retval[2*k*timesteps_*robot_kdl_->dof+robot_kdl_->dof*(timesteps_)+robot_kdl_->dof*(i)+l]+=wo*o_gradient[l];
      
        }
      }
    }
    
    fill(p_gradient.begin(), p_gradient.end(), 0.0);
    fill(o_gradient.begin(), o_gradient.end(), 0.0);
        
  }
  return retval;
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

  // RR inequality constraints:
  // position constraint
  for (int k=0;k<data_arr_[3].size();++k)
  {
    for(int i=0;i<timesteps_;++i)
    {
      for(int j=0;j<robot_kdl_->dof;++j)
      {
        retval.emplace_back(1+m_nec+robot_kdl_->dof*(timesteps_)+2*k*timesteps_+i,i*robot_kdl_->dof+j);
      }
    }
    //for (int k=0;k<data_arr_[3].size();++k)
    // orientation constraint
    for(int i=0;i<timesteps_;++i)
    {
      for(int j=0;j<robot_kdl_->dof;++j)
      {
        retval.emplace_back(1+m_nec+robot_kdl_->dof*timesteps_+2*k*timesteps_+timesteps_+i,i*robot_kdl_->dof+j);
      }
    }
  }
  
  /*
  for(int i=0;i<retval.size();++i)
  {
    cerr<<retval[i].first<<","<<retval[i].second<<endl;
  }
  */
  return retval;
}
#if LL4MA_COLLISION_MODEL==true

vector<double> optProblem::collisionConstraints(vector<double> x) const
{
  vector<double> retval;
  return retval;  
}

vector<double> optProblem::collisionGradient(vector<double> x) const
{
  vector<double> retval;
  
  return retval;
}
#endif
vector<double> optProblem::debugFunction(const vector<double> x) const
{
  double cost_val=0.0;
  // relaxed rigidity constraints:
  Eigen::VectorXd q;
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
    Eigen::Map<Eigen::VectorXd> pos_offset(&p_off[0],p_off.size());
    pos_offsets.emplace_back(pos_offset);
  }
  double wo,wr,wd,wi;
  wo=data_arr_[1][0];
  wr=data_arr_[1][1];
  wd=data_arr_[1][2];
  wi=data_arr_[1][3];
  q.resize(robot_kdl_->dof);

  vector<double> o_err;

  double tot_pos_cost=0.0;
  double debug_cost=0.0;
  vector<Eigen::VectorXd> j_poses;
  // store finger pose for min acceleration cost:
  vector<double> init_joints=joints_0;


  Eigen::Map<Eigen::VectorXd>j_0(&init_joints[0],init_joints.size());

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
  //cerr<<r_f_poses.size()<<endl;
  j_poses.push_back(q);
  j_poses.push_back(q);
   
  Eigen::VectorXd acc_cost(16),ee_pose;
  

  Eigen::MatrixXd f_mat,thumb_mat;
  
  for(int i=0; i<timesteps_;++i)
  {
    double position_cost=0.0;
    double orientation_cost=0.0;

    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_->dof;++j)
    {
      q[j]=x[(i)*robot_kdl_->dof+j];
    }
    robot_kdl_->getFK(rigid_f,q.segment(rigid_f*4,4),ee_pose,true);
    

    // Relaxed-Rigidity costs:
    // get current thumb mat:
    robot_kdl_->getFK(rigid_f,q.segment(rigid_f*4,4),thumb_mat);
    
    for(int k=0;k<data_arr_[3].size();++k)
    {
      int f=data_arr_[3][k];
      if(f!=rigid_f)
      {
        // Position:
        robot_kdl_->getFK(f,q.segment(f*4,4),f_mat);
        Eigen::MatrixXd des_mat=thumb_mat*data_mats_[f];


	// compute difference between the two frames:
        
	
        Eigen::VectorXd pose_diff(6);
        
        pose_diff[0]=des_mat(0,3)-f_mat(0,3);
        pose_diff[1]=des_mat(1,3)-f_mat(1,3);
        pose_diff[2]=des_mat(2,3)-f_mat(2,3);
        
        
        // Compute orientation error:
        robot_kdl_->euler_diff(f_mat,des_mat,o_err);
        
        pos_err=pose_diff.segment(0,3)*1.0;
        
        orient_err[1]=o_err[1];
        orient_err[0]=0.0;
        orient_err[2]=0.0;
        position_cost+=max(pos_err.squaredNorm(),wr)-wr;
        orientation_cost+=orient_err.squaredNorm();
      }
      
    }
    cost_val+=pow(position_cost,2);//position_cost;
    //cerr<<position_cost<<endl;
    //cost_val=position_cost;//pow(position_cost,2);//pow((max(position_cost,wr)-wr),2);
    //vel_const[robot_kdl_->dof*timesteps_+timesteps_+i]=pow(orientation_cost,2);
  }
  vector<double> cost_grad={cost_val};
 
  return cost_grad;

}
vector<double> optProblem::debugGrad(vector<double> state) const
{
  vector<double> retval(robot_kdl_->dof,0.0);
  using std::placeholders::_1;

  std::function<vector<double>(vector<double>)> f = std::bind(&optProblem::debugFunction,this,_1);

  //  retval=pagmo::estimate_gradient(std::bind(&optProblem::forceBalance,this,_1),x);
  retval=pagmo::estimate_gradient(f,state);
  return retval; 

}

}
