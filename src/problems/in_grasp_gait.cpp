#include "optimization_pkg/problems/opt_problem.hpp"
//#include <tf/transform_datatypes.h>

using namespace opt_problems;

optProblem::optProblem(int dummy)
{
  std::cerr<<"optProblem dummy constructor is initialized, fix me!!!"<<std::endl;
}

// Initializer
optProblem::optProblem(int time_steps,manipulator_kdl::robotKDL robot_kdl,vector<vector<double>> data_arr,vector<double> vel_limits, vector<double> init_joint_angles)
{
  cerr<<"Called wrong constructor!!"<<endl;
}


optProblem::optProblem(int time_steps,manipulator_kdl::robotKDL robot_kdl,collisionChecker c_check, vector<vector<double>> data_arr,vector<double> vel_limits,vector<double> init_joint_angles)
{
  //cerr<<"setting up opt"<<endl;

  m_dim=robot_kdl.dof*time_steps;
  m_nec=0;// 
  m_nic=0;//
  timesteps_=time_steps;
  robot_kdl_=robot_kdl;

  joints_0=init_joint_angles;
  data_arr_=data_arr;
  
  //cerr<<"Computing object frame"<<endl;
  
  // Initialize object frame
 
  tf::Quaternion q(data_arr_[0][3], data_arr_[0][4], data_arr_[0][5], data_arr_[0][6]);
  tf::Matrix3x3 R(q);
  Eigen::MatrixXd obj_mat(4,4);
  obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    obj_mat(i,0)=R[i][0];
    obj_mat(i,1)=R[i].y();
    obj_mat(i,2)=R[i].z();
  }
  obj_mat(0,3)=data_arr_[0][0];
  obj_mat(1,3)=data_arr_[0][1];
  obj_mat(2,3)=data_arr_[0][2];

  // Get rigid_finger matrix
  int rigid_finger=data_arr_[2][0];
  vector<double> rigidf_joints(&joints_0[rigid_finger*4],&joints_0[rigid_finger*4+4]);
  
  Eigen::MatrixXd thumb_mat=robot_kdl.getFKEigen(rigid_finger,rigidf_joints);
  // Compute T_mat from rigid_finger to object
  Eigen::MatrixXd obj_offset=thumb_mat.inverse()*obj_mat;
  //cerr<<"Got object offset!"<<endl;
  // Store in data_mats:
  data_mats.push_back(obj_offset);
  data_mats.push_back(obj_offset.inverse());
  c_checker=c_check;
  des_poses.resize(1);
  for(int i=0;i<1;++i)
  {    
    Map<Eigen::VectorXd> des_pose(&data_arr[data_arr.size()-1][0],data_arr[data_arr.size()-1].size());// only position
    des_poses[i]=des_pose;
  }
  // store desired obj pose as T mat:

}



double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;

  vector<double> q;
  vector<double> ee_pose_arr,obj_pose_arr;
  Eigen::VectorXd obj_pose_err;
  Eigen::MatrixXd thumb_mat,obj_mat;
  obj_pose_err.resize(6);
  obj_pose_err.setZero();
  
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
  wd=data_arr_[1][0];
  wr=data_arr_[1][1];
  wo=data_arr_[1][2];
  vector<double> des_f_arr;
  des_f_arr.resize(6);
  q.resize(robot_kdl_.dof,0.0);
  Eigen::MatrixXd des_f_mat;
  for(int i=0; i<timesteps_;i++)
  {
    double position_cost=0.0;
    double orientation_cost=0.0;
    
    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }

    // Rigid finger cost:    
    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);
    
    thumb_mat=robot_kdl_.getFKEigen(rigid_f,f_q);
    ee_pose_arr=robot_kdl_.frame_to_pose_(thumb_mat,true);
    obj_mat=thumb_mat*data_mats[0];
    obj_pose_arr=robot_kdl_.frame_to_pose_(obj_mat,true);
    
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);
    Map<Eigen::VectorXd> obj_pose(&obj_pose_arr[0],6);

    // object pose error:
    
    // convert desired object pose to finger pose:
    //cerr<<"Computing desired ee pose: "<<endl;
    des_f_mat=robot_kdl_.pose_to_frame(des_poses[0])*data_mats[1];

    des_f_arr=robot_kdl_.frame_to_pose_(des_f_mat,true);
    Map<Eigen::VectorXd> ee_des_pose(&des_f_arr[0],6);
    // compute error in finger pose:
    
    
    obj_pose_err=ee_des_pose-ee_pose;

    //obj_pose_err[3]=0.0;
    obj_pose_err[4]=0.0;
    obj_pose_err[5]=0.0;
    
    
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
      cost_val+=wd*obj_pose_err.lpNorm<1>()+wr*position_cost+wo*orientation_cost;
    }
    else
    {
      cost_val+=wr*position_cost+wo*orientation_cost;
    }
    
  }
  //cerr<<"Cost computed"<<endl;
  //cerr<<cost_val<<endl;
  return cost_val;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  // limiting velocity at joints
  vector<double> vel_const(m_nic,0.0);
  
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
  Eigen::MatrixXd des_f_mat;
  vector<double> des_f_arr;
  des_f_arr.resize(6);
  double wo,wr,wd;
  wd=data_arr_[1][0];
  wr=data_arr_[1][1];
  wo=data_arr_[1][2];
  vector<double> retval(m_dim,0.0);

  vector<double> q;
  vector<double> ee_pose_arr,obj_pose_arr;

  Eigen::VectorXd obj_pose_err;
  Eigen::MatrixXd thumb_mat,obj_mat;
  obj_pose_err.resize(6);
  obj_pose_err.setZero();

  Eigen::VectorXd finger_gradient;
  vector<double> gradient;
  gradient.resize(robot_kdl_.dof,0.0);

  
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
  

  Eigen::MatrixXd link_J;  

  for(int i=0;i<timesteps_;++i)
  {
    
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(i)*robot_kdl_.dof+j];
    }


    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);

    
    thumb_mat=robot_kdl_.getFKEigen(rigid_f,f_q);
    ee_pose_arr=robot_kdl_.frame_to_pose_(thumb_mat,true);
    obj_mat=thumb_mat*data_mats[0];
    obj_pose_arr=robot_kdl_.frame_to_pose_(obj_mat,true);
    
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);
    Map<Eigen::VectorXd> obj_pose(&obj_pose_arr[0],6);
    
    

    // Convert desired object pose to finger pose:
    des_f_mat=robot_kdl_.pose_to_frame(des_poses[0])*data_mats[1];
    des_f_arr=robot_kdl_.frame_to_pose_(des_f_mat,true);
    Map<Eigen::VectorXd> ee_des_pose(&des_f_arr[0],6);
    // compute error in finger pose:
    
    obj_pose_err=ee_des_pose-ee_pose;
    //obj_pose_err[3]=0.0;
    obj_pose_err[4]=0.0;
    obj_pose_err[5]=0.0;
    if(i==timesteps_-1)
    {
      //cerr<<obj_pose_err.transpose()<<endl;
      robot_kdl_.getPseudoInverse(robot_kdl_.getJacobian(rigid_f,f_q),link_J,0.0001);

      finger_gradient=-1.0*wd*link_J*obj_pose_err;
      //cerr<<finger_gradient.transpose()<<endl;
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
        pos_err[4]=1.0*pos_err[4]*wo;
        pos_err[5]=0.0;
        robot_kdl_.getPseudoInverse(robot_kdl_.getJacobian(f,f_joints),link_J,0.0001);
        
        finger_gradient=-1.0*link_J*pos_err;
        for (int l=0;l<4;++l)
        {
          gradient[f*4+l]+=finger_gradient[l];
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
  
  /*
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
  */
  return retval;
}
vector<vector<double>> optProblem::bounds() const
{
  vector<vector<double>> bound;
  vector<double> up_bounds;
  vector<double> low_bounds;
  for(int i=0;i<timesteps_;i++)
  {
    up_bounds.insert(up_bounds.end(), robot_kdl_.up_bounds.begin(), robot_kdl_.up_bounds.end());
    low_bounds.insert(low_bounds.end(), robot_kdl_.low_bounds.begin(), robot_kdl_.low_bounds.end());
  }

  
  bound.resize(2);
  bound[0]=low_bounds;
  bound[1]=up_bounds;
  return bound;
  
}


