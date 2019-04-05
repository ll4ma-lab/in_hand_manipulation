#include "optimization_pkg/problems/opt_problem_full.hpp"
#define COLL_CHECK false
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
  m_nec=robot_kdl.dof;// 
  m_nic=(time_steps-1)*(1+robot_kdl.dof);//
  //cerr<<m_nic<<endl;
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
  // Compute contact point transformation matrices w.r.t object:
  Eigen::MatrixXd p_T_pt,o_T_pt;
  
  for(int i=0;i<data_arr_[3].size();++i)
  {
    // Get contact point in palm frame:
    vector<double> pt=data_arr_[3*4+i];
    Eigen::VectorXd pt_(7);
    pt_.setZero();
    pt_[0]=pt[0];
    pt_[1]=pt[1];
    pt_[2]=pt[2];
    pt_[6]=1.0;
    // o_T_pt=o_T_p * p_T_pt
    p_T_pt=robot_kdl_.pose_to_frame(pt_);

    o_T_pt=obj_mat*p_T_pt;
    data_mats.push_back(o_T_pt);
  }
  cerr<<"Stored contact point locations!"<<endl;
  vel_lim=vel_limits[0];
}



double optProblem::objFunction(const vector<double> x) const
{
  // Cost function is on reaching a desired cartesian pose 
  double cost_val=0.0;
  double reach_val=0.0;
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
  for(int t=0; t<timesteps_;t++)
  {
    double position_cost=0.0;
    double orientation_cost=0.0;
    
    // Getting joint angles for current timestep:
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(t)*robot_kdl_.dof+j];
    }

    
    // Rigid finger cost:    
    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);
    
    thumb_mat=robot_kdl_.getFKEigen(rigid_f,f_q);
    ee_pose_arr=robot_kdl_.frame_to_pose_(thumb_mat,true);
    obj_mat=thumb_mat*data_mats[0];
    obj_pose_arr=robot_kdl_.frame_to_pose_(obj_mat,true);
    
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);
    Map<Eigen::VectorXd> obj_pose(&obj_pose_arr[0],6);
    Map<Eigen::VectorXd> obj_pos(&obj_pose_arr[0],3);

    // object pose error:
    
    // Transform desired contact points to new object pose:
    vector<vector<double>> pts;
    for(int i=0;i<data_arr_[3].size();++i)
    {
      // o_T_pt
      Eigen::MatrixXd T=obj_mat*data_mats[2+i];
      vector<double> pose=robot_kdl_.frame_to_pose_(T,true);
      vector<double> pt(3);
      pt[0]=pose[0];
      pt[1]=pose[1];
      pt[2]=pose[2];      
      pts.push_back(pt);
    }
    //if(t==timesteps_-1)
    {
      // Get signed distance between reachable workspaces and the desired contact points:
      vector<int> link_idx={13,14,15,16};
      vector<collision_data> c_data=c_checker.check_pt_collisions(pts,link_idx);
      Eigen::VectorXd delta_obj_pos(3);
      delta_obj_pos.setZero();
      
      for(int i=0;i<c_data.size();++i)
      {
        if(c_data[i].distance<0.0)
        {
        reach_val+=-1.0*c_data[i].distance;
        Map<Eigen::VectorXd> point(&c_data[i].point[0],3);
        Map<Eigen::VectorXd> dir(&c_data[i].dir[0],3);
        double p_d;
        p_d=c_data[i].distance;
        Eigen::VectorXd delta_p;        
        delta_p=obj_pos-(point-p_d*dir);
        delta_obj_pos+=delta_p;
        }
      }
      
      obj_pose[0]+=delta_obj_pos[0]/4.0;
      obj_pose[1]+=delta_obj_pos[1]/4.0;
      obj_pose[2]+=delta_obj_pos[2]/4.0;
      
      des_f_mat=robot_kdl_.pose_to_frame(obj_pose)*data_mats[1];
      des_f_arr=robot_kdl_.frame_to_pose_(des_f_mat,true);
      Map<Eigen::VectorXd> ee_des_pose(&des_f_arr[0],6);
      // compute error in finger pose:
      
      obj_pose_err=ee_des_pose-ee_pose;
      
      
      obj_pose_err[3]=0.0;
      obj_pose_err[4]=0.0;
      obj_pose_err[5]=0.0;
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
        pos_err=pose_diff.segment(0,3);
        orient_err=pose_diff.segment(3,3);
        orient_err[0]=0.0;
        orient_err[2]=0.0;
        position_cost+=pos_err.lpNorm<1>();
        orientation_cost+=orient_err.lpNorm<1>();
      }
    }
    
    //if(t==timesteps_-1)
    {

      cost_val+=wd*obj_pose_err.lpNorm<1>()+wr*position_cost+wo*orientation_cost;
    }
    //else
    {
      //cost_val+=wr*position_cost+wo*orientation_cost;
    }
    
  }
  //cerr<<cost_val<<endl;
  return cost_val;
}
vector<double> optProblem::inEqConstraints(vector<double> x) const
{
  // limiting velocity at joints

  vector<double> ret(m_nic,0.0);
  //return ret;
  
  for (int i=0; i<timesteps_-1; ++i)
  {  
    for(int j=0;j<robot_kdl_.dof;++j)
    {
      if(std::abs(x[(i+1)*robot_kdl_.dof+j]-  x[(i)*robot_kdl_.dof+j])>vel_lim)
      {
        ret[(timesteps_-1)+(i)*robot_kdl_.dof+j]=std::abs(x[(i+1)*robot_kdl_.dof+j]-  x[(i)*robot_kdl_.dof+j])-vel_lim;
      }
    }
  }
  if(COLL_CHECK==false)
  {
    return ret;
  }
  cerr<<"Computing collisions!"<<endl;
  vector<vector<collision_data>> c_data;

  vector<double> palm_pose;
  palm_pose.resize(7,0.0);
  palm_pose[6]=1.0;
  vector<double> q;
  q.resize(16);
  vector<double> obj_pose_arr;
  Eigen::MatrixXd thumb_mat,obj_mat;
  
  
  int rigid_f=int(data_arr_[2][0]);// rigid_finger
  double collision_const=0.0;

  // Collision constraint:
  // Check for collisions between object and hand:
  for(int t=1; t<timesteps_;t++)
  {
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(t)*robot_kdl_.dof+j];
    }

    // Update object pose in collision checker:
    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);
    thumb_mat=robot_kdl_.getFKEigen(rigid_f,f_q);

    obj_mat=thumb_mat*data_mats[0];
    obj_pose_arr=robot_kdl_.frame_to_pose_(obj_mat,false);

    c_checker.update_env_pose(obj_pose_arr);

 
    vector<int> link_idx={1,2,4,5,7,8,10,11};
    vector<int> joint_idx_low={0,0,4,4,8,8,12,12};
    vector<int> joint_idx_up={2,3,6,7,10,11,15,16};
    vector<int> kdl_link_idx={0,1,3,4,6,7,9,10};
    
    vector<vector<double>> l_poses;

    l_poses.push_back(palm_pose);
    
    // Get finger link poses
    for(int i=0;i<link_idx.size();i++)
    {
      vector<double> f_q(&q[joint_idx_low[i]],&q[joint_idx_up[i]]);
      vector<double> l_pose_arr=robot_kdl_.getLinkPose(f_q,kdl_link_idx[i]);
      l_poses.push_back(l_pose_arr);
    }
    
    link_idx.insert(link_idx.begin(),0);
    // Update link poses in collision checker:
    c_checker.update_robot_state(l_poses,link_idx);
    // Check for collisions:
    c_data=c_checker.check_collisions(l_poses,link_idx);
    for(int i =0; i<1; ++i)
    {
      for(int l=0;l<c_data[i].size();++l)
      {
        if(c_data[i][l].distance>0.0)
        {
          //cerr<<"link: "<<link_idx[i]<<endl;
          collision_const+=c_data[i][l].distance;
        }
      }
    }
    ret[(t-1)]=collision_const; 
  }
  //c_checker.publish_env_objects(-1,c_checker.pub_);
  //c_checker.publish_robot_objects();
  
  
  return ret;
}
vector<double> optProblem::EqConstraints(vector<double> x) const
{

  vector<double> ret(m_nec,0.0);
  // initial joint angles constraint:
  for (int i=0;i<robot_kdl_.dof;++i)
  {
    ret[i]=joints_0[i]-x[i];
  }
  
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

  for(int t=0;t<timesteps_;++t)
  {
    
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(t)*robot_kdl_.dof+j];
    }


    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);

    
    thumb_mat=robot_kdl_.getFKEigen(rigid_f,f_q);
    ee_pose_arr=robot_kdl_.frame_to_pose_(thumb_mat,true);
    obj_mat=thumb_mat*data_mats[0];
    obj_pose_arr=robot_kdl_.frame_to_pose_(obj_mat,true);
    
    Map<Eigen::VectorXd> ee_pose(&ee_pose_arr[0],6);
    Map<Eigen::VectorXd> obj_pose(&obj_pose_arr[0],6);
    Map<Eigen::VectorXd> obj_pos(&obj_pose_arr[0],3);
    // object pose error:
    
    // Transform desired contact points to new object pose:
    vector<vector<double>> pts;
    for(int i=0;i<data_arr_[3].size();++i)
    {
      // o_T_pt
      Eigen::MatrixXd T=obj_mat*data_mats[2+i];
      vector<double> pose=robot_kdl_.frame_to_pose_(T,true);
      vector<double> pt(3);
      pt[0]=pose[0];
      pt[1]=pose[1];
      pt[2]=pose[2];      
      pts.push_back(pt);
    }
    
    // Get signed distance between reachable workspaces and the desired contact points:
    vector<int> link_idx={13,14,15,16};
    vector<collision_data> c_data=c_checker.check_pt_collisions(pts,link_idx);
    Eigen::VectorXd delta_obj_pos(3);
    delta_obj_pos.setZero();
    
    for(int i=0;i<c_data.size();++i)
    {
      if(c_data[i].distance<0.0)
      {
        Map<Eigen::VectorXd> point(&c_data[i].point[0],3);
        Map<Eigen::VectorXd> dir(&c_data[i].dir[0],3);
        double p_d;
        p_d=c_data[i].distance;
        Eigen::VectorXd delta_p;        
        delta_p=obj_pos-(point-p_d*dir);
        delta_obj_pos+=delta_p;
      } 
    }
    obj_pose[0]+=delta_obj_pos[0]/4.0;
    obj_pose[1]+=delta_obj_pos[1]/4.0;
    obj_pose[2]+=delta_obj_pos[2]/4.0;

    // Convert desired object pose to finger pose:
    des_f_mat=robot_kdl_.pose_to_frame(obj_pose)*data_mats[1];
    des_f_arr=robot_kdl_.frame_to_pose_(des_f_mat,true);
    Map<Eigen::VectorXd> ee_des_pose(&des_f_arr[0],6);
    // compute error in finger pose:
    
    obj_pose_err=ee_des_pose-ee_pose;
    obj_pose_err[3]=0.0;//obj_pose_err[3]*-1.0;
    obj_pose_err[4]=0.0;//obj_pose_err[4]*-1.0;
    obj_pose_err[5]=0.0;//obj_pose_err[5]*1.0;
    //if(t==timesteps_-1)
    {
      //cerr<<obj_pose_err.transpose()<<endl;
      //robot_kdl_.getPseudoInverse(robot_kdl_.getJacobian(rigid_f,f_q),link_J,0.0001);
      Eigen::MatrixXd l_J=robot_kdl_.getJacobian(rigid_f,f_q);
      link_J=l_J.transpose();

      finger_gradient=-1.0*wd*link_J*obj_pose_err;
      for (int k=0;k<4;++k)
      {
        gradient[rigid_f*4+k]+=finger_gradient[k];
      }
    } 

    Eigen::MatrixXd rigid_J=link_J;
    
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
        Eigen::VectorXd thumb_gradient(4);
        thumb_gradient.setZero();
        Eigen::VectorXd pose_diff=ee_pose-f_pose-pos_offsets[k];
        pos_err=pose_diff;//.segment(0,3);
        //orient_err=pose_diff;//.segment(3,3);
        pos_err[0]=wr*pos_err[0];
        pos_err[1]=wr*pos_err[1];
        pos_err[2]=wr*pos_err[2];
        pos_err[3]=0.0;
        pos_err[4]=-1.0*pos_err[4]*wo;
        pos_err[5]=0.0;
        Eigen::MatrixXd l_J=robot_kdl_.getJacobian(f,f_joints);
        link_J=l_J.transpose();
        finger_gradient=-1.0*link_J*pos_err;
        thumb_gradient=1.0*rigid_J*pos_err;
        for (int l=0;l<4;++l)
        {
          gradient[f*4+l]+=finger_gradient[l];
          //gradient[rigid_f*4+l]+=thumb_gradient[l];
        }

      }
    }
    for (int j=0;j<robot_kdl_.dof;++j)
    {
      retval[robot_kdl_.dof*(t)+j]+=gradient[j];
    }
    /*
    if(t==2)
    {
      cerr<<"Gr: "<<endl;
      for(int p=0;p<gradient.size();++p)
      {
        cerr<<gradient[p]<<" ";
      }
      cerr<<endl;
    }
    */
    fill(gradient.begin(), gradient.end(), 0.0);

  }

  return retval;
}

vector<double> optProblem::inEqGradient(vector<double> x) const
{
  vector<double> retval(m_dim*m_nic,0.0);
  //return retval;
  for(int i=0;i<timesteps_-1;++i)
  {
      for(int j=0;j<robot_kdl_.dof;++j)
      {
        if(std::abs(x[(i+1)*robot_kdl_.dof+j]-  x[(i)*robot_kdl_.dof+j])>vel_lim)
        {
          if(x[(i+1)*robot_kdl_.dof+j]-x[(i)*robot_kdl_.dof+j]<0.0)
          {
            retval[((i)*robot_kdl_.dof+j+(timesteps_-1))*m_dim+(i+1)*robot_kdl_.dof+j]=-1.0;
            if(i!=0)
            {
              retval[((i)*robot_kdl_.dof+j+(timesteps_-1))*m_dim+(i)*robot_kdl_.dof+j]=1.0;
            }
          }
          else
          {
            retval[((i)*robot_kdl_.dof+j+(timesteps_-1))*m_dim+(i+1)*robot_kdl_.dof+j]=1.0;
            if(i!=0)
            {
              retval[((i)*robot_kdl_.dof+j+(timesteps_-1))*m_dim+(i)*robot_kdl_.dof+j]=-1.0;
            }
          }
        }
      }
  }


  
  if(COLL_CHECK==false)
  {
    return retval;
  }

  vector<vector<collision_data>> c_data;

  vector<double> palm_pose;
  palm_pose.resize(7,0.0);
  palm_pose[6]=1.0;
  vector<double> q;
  q.resize(16);
  vector<double> obj_pose_arr;
  Eigen::MatrixXd thumb_mat,obj_mat;
  
  
  int rigid_f=int(data_arr_[2][0]);// rigid_finger
  double collision_const=0.0;

  // Collision constraint:
  // Check for collisions between object and hand:
  for(int t=1; t<timesteps_-1;t++)
  {
    for(int j=0; j<robot_kdl_.dof;++j)
    {
      q[j]=x[(t)*robot_kdl_.dof+j];
    }

    // Update object pose in collision checker:
    vector<double> f_q(&q[rigid_f*4],&q[rigid_f*4+4]);
    thumb_mat=robot_kdl_.getFKEigen(rigid_f,f_q);

    obj_mat=thumb_mat*data_mats[0];
    obj_pose_arr=robot_kdl_.frame_to_pose_(obj_mat,false);

    c_checker.update_env_pose(obj_pose_arr);

    vector<int> link_idx={1,2,3,4,5,6,7,8,9,10,11,12};
    //vector<int> link_idx={1,2,4,5,7,8,10,11};
    vector<int> joint_idx_low={0,0,0,4,4,4,8,8,8,12,12,12};
    vector<int> joint_idx_up={2,3,4,6,7,8,10,11,12,15,16,16};
    vector<int> kdl_link_idx={0,1,2,3,4,5,6,7,8,9,10,11};
    vector<int> f_idx={-1,0,0,0,1,1,1,2,2,2,3,3,3};
    vector<vector<double>> l_poses;

    l_poses.push_back(palm_pose);
    
    // Get finger link poses
    for(int i=0;i<link_idx.size();i++)
    {
      vector<double> f_q(&q[joint_idx_low[i]],&q[joint_idx_up[i]]);
      vector<double> l_pose_arr=robot_kdl_.getLinkPose(f_q,kdl_link_idx[i]);
      l_poses.push_back(l_pose_arr);
    }
    
    link_idx.insert(link_idx.begin(),0);
    // Update link poses in collision checker:
    c_checker.update_robot_state(l_poses,link_idx);
    // Check for collisions:
    c_data=c_checker.check_collisions(l_poses,link_idx);

    // Move object if it is in collision with palm:
    f_q=vector<double>(&q[rigid_f*4],&q[rigid_f*4+4]);
    
    Map<Eigen::VectorXd> obj_pose(&obj_pose_arr[0],3);
    

    
    Eigen::MatrixXd rigid_J;
    Eigen::MatrixXd l_J=robot_kdl_.getJacobian(rigid_f,f_q).block(0,0,3,f_q.size());
    rigid_J=l_J.transpose();
        
    //robot_kdl_.getPseudoInverse(robot_kdl_.getJacobian(rigid_f,f_q).block(0,0,3,f_q.size()), rigid_J,0.0001);
    Eigen::VectorXd finger_gradient(4);
    finger_gradient.setZero();
    vector<double> delta_pose;
    delta_pose.resize(6,0.0);
    int i=0;
    for(int l=0;l<c_data[i].size();++l)
    {
      if(c_data[i][l].distance>0.0)
      {
        // compute delta pose:
        Map<Eigen::VectorXd> point(&c_data[i][l].point[0],3);
        Map<Eigen::VectorXd> dir(&c_data[i][l].dir[0],3);
        double p_d;
        p_d=c_data[i][l].distance;
        Eigen::VectorXd delta_p;
        //delta_p=obj_pose+(point-p_d*dir);
        delta_p=obj_pose-(point+p_d*dir);
        delta_pose[0]=delta_p[0];
        delta_pose[1]=delta_p[1];
        delta_pose[2]=delta_p[2];
        // Convert object pose to finger pose:
        // p_T_ee=p_T_o*o_T_ee
        vector<double> des_f_arr=robot_kdl_.frame_to_pose_(robot_kdl_.pose_to_frame(delta_pose)*data_mats[1],true);
        Map<Eigen::VectorXd> delta_pt(&des_f_arr[0],3);
        // Add to finger gradient:
        finger_gradient.segment(0,f_q.size())+=-1.0*rigid_J*delta_pt;
      }
    }
    // add to hand gradient:
    for(int l=0;l<robot_kdl_.f_dof;++l)
    {
      retval[m_dim*(t)+robot_kdl_.f_dof*rigid_f+l]+=finger_gradient[l];
    }
    

    for(int i=1; i<c_data.size(); ++i)
    {
      // TODO: Fix complex indexing else code will be unreadable later!!
      // i-1 since kdl doesn't have palm
      vector<double> f_q(&q[joint_idx_low[i-1]],&q[joint_idx_up[i-1]]);
      Map<Eigen::VectorXd> link_pose(&l_poses[i][0],3);
      Eigen::MatrixXd link_J=robot_kdl_.getLinkJacobian(kdl_link_idx[i-1],f_q).block(0,0,3,f_q.size()).transpose();
      
      //robot_kdl_.getPseudoInverse(robot_kdl_.getLinkJacobian(kdl_link_idx[i-1],f_q).block(0,0,3,f_q.size()), link_J,0.0001);
      Eigen::VectorXd finger_gradient(4);
      finger_gradient.setZero();
      
      for(int l=0;l<c_data[i].size();++l)
      {
        if(c_data[i][l].distance>0.0)
        {
          // compute delta pose:
          Map<Eigen::VectorXd> point(&c_data[i][l].point[0],3);
          Map<Eigen::VectorXd> dir(&c_data[i][l].dir[0],3);
          double p_d;
          p_d=c_data[i][l].distance;
          Eigen::VectorXd delta_p;        
          delta_p=link_pose-(point-p_d*dir);
          // Add to finger gradient:
          finger_gradient.segment(0,f_q.size())+=-1.0*link_J*delta_p;
        }
      }
      // add to hand gradient:
      for(int l=0;l<robot_kdl_.f_dof;++l)
      {
        retval[robot_kdl_.f_dof*f_idx[i]+l]+=finger_gradient[l];
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
    retval[i*m_dim+i]=-1.0;
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

  // Equality constraints:

  for(int i=0;i<robot_kdl_.dof;++i)
  {
    grad.push_back(eq_grad[i*m_dim+i]);
  }

  
  // Indexing inequality gradient for collisions:

  
  //  collision constraint:
  for(int t=1;t<timesteps_;++t)
  {
    for(int j=0;j<4;++j)
    {
      grad.push_back(ineq_grad[t*m_dim+3*4+j]);
    }
  }

  // velocity gradient:
  for(int t=0;t<timesteps_-1;++t)
  {
    for(int j=0;j<robot_kdl_.dof;++j)
    {
      grad.push_back(ineq_grad[((t*robot_kdl_.dof)+j+(timesteps_-1))*m_dim+t*robot_kdl_.dof+j]);
    }
    for(int j=0;j<robot_kdl_.dof;++j)
    {
      grad.push_back(ineq_grad[((t*robot_kdl_.dof)+j+(timesteps_-1))*m_dim+(t+1)*robot_kdl_.dof+j]);
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
  for(int i=0;i<robot_kdl_.dof;++i)
  {
    retval.emplace_back(i+1,i);
  }

  // Collision gradient indices:

  // inEquality Gradients:
  
  // Collision gradient:
  for(int t=0;t<timesteps_-1;++t)
  {
    for(int j=3*4;j<3*4+4;++j)
    {
      retval.emplace_back(1+m_nec+(t),j);
    }
   
  }
  

  // Velocity gradient:
  for(int t=0;t<timesteps_-1;++t)
  {
    for(int j=0;j<robot_kdl_.dof;++j)
    {
      retval.emplace_back(1+m_nec+(timesteps_-1)+t,(t)*robot_kdl_.dof+j);
    }
    for(int j=0;j<robot_kdl_.dof;++j)
    {
      retval.emplace_back(1+m_nec+(timesteps_-1)+t,(t+1)*robot_kdl_.dof+j);
    }
  }
  
  return retval;
}
*/
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


