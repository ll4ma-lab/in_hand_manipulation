#define SPARSE_GRADIENT true
// optimization:
#include <in_grasp_manipulation/problems/rolling_problem.hpp>
#include <in_hand_manipulation/pagmo_opt/trajectory_optimization.hpp>

//#include <ll4ma_planner/wrapper/pagmo_wrapper.hpp>
// ROS:
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include <Eigen/Geometry> 

// ROS service for in grasp manipulation:
#include "in_grasp_manipulation/InGraspManipulation.h"
using namespace pagmo;

bool in_grasp_call(in_grasp_manipulation::InGraspManipulation::Request &req,in_grasp_manipulation::InGraspManipulation::Response &res)
{
  // Allegro URDF file:
  
  string urdf_file="/home/bala/catkin_ws/src/in_hand_manipulation/in_grasp_manipulation/urdf/allegro.urdf";

  // Load allegro kdl object:
  vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics

  manipulator_kdl::robotKDL allegro_kdl(urdf_file,base_names,ee_names,g_vec);
  allegro_kdl.urdfParser();
  // Getting parameters from service request:

  Map<Eigen::VectorXd> joints_0(&req.init_joints.position[0],req.init_joints.position.size());
  Eigen::VectorXd final_joints;
  if(req.final_joints.position.size()!=0)
  {
    Map<Eigen::VectorXd> final_joints(&req.final_joints.position[0],req.final_joints.position.size());
  }
  else
  {
    Map<Eigen::VectorXd> final_joints(&req.init_joints.position[0],req.init_joints.position.size());
  }

  vector<Eigen::MatrixXd> data_mat;
  vector<vector<double>> data_arr;

  
  // Get initial object to thumb frame:
  tf::Pose init_pose;
  tf::poseMsgToTF(req.object_pose,init_pose);
  // init_pose->4x4
  tf::Quaternion q(req.object_pose.orientation.x,req.object_pose.orientation.y,req.object_pose.orientation.z,req.object_pose.orientation.w);
  tf::Matrix3x3 R(q);
  
  Eigen::MatrixXd obj_mat(4,4);
  obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    obj_mat(i,0)=R[i][0];
    obj_mat(i,1)=R[i][1];
    obj_mat(i,2)=R[i][2];
  }
  obj_mat(0,3)=req.object_pose.position.x;
  obj_mat(1,3)=req.object_pose.position.y;
  obj_mat(2,3)=req.object_pose.position.z;

  

  Eigen::MatrixXd thumb_mat;
  allegro_kdl.getFK(3,joints_0.segment(12,4),thumb_mat);
  
  Eigen::MatrixXd obj_offset=thumb_mat.inverse()*obj_mat;
  
  tf::Pose des_pose_tf;
  tf::poseMsgToTF(req.desired_object_pose,des_pose_tf);
  //cerr<<
  // convert desired object pose to thumb pose:
  R=des_pose_tf.getBasis();
  
  Eigen::MatrixXd des_obj_mat(4,4);
  des_obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    des_obj_mat(i,0)=R[i][0];
    des_obj_mat(i,1)=R[i][1];
    des_obj_mat(i,2)=R[i][2];
  }
  des_obj_mat(0,3)=req.desired_object_pose.position.x;
  des_obj_mat(1,3)=req.desired_object_pose.position.y;
  des_obj_mat(2,3)=req.desired_object_pose.position.z;
  //cerr<<des_obj_mat(0,3)<<endl;
  Eigen::MatrixXd des_thumb_mat=des_obj_mat*obj_offset.inverse();

  // Get des pose from mat:
  //thumb_joints[0]=0.8;
  //vector<double> des_pose_thumb=allegro_kdl.getFK(3,thumb_joints,true);
  vector<double> des_pose;
  des_pose.resize(6);
  des_pose[0]=des_thumb_mat(0,3);
  des_pose[1]=des_thumb_mat(1,3);
  des_pose[2]=des_thumb_mat(2,3);
  // Get euler xyz from rotation matrix:
  tf::Matrix3x3 R_(des_thumb_mat(0,0),des_thumb_mat(0,1),des_thumb_mat(0,2),
                   des_thumb_mat(1,0),des_thumb_mat(1,1),des_thumb_mat(1,2),
                   des_thumb_mat(2,0),des_thumb_mat(2,1),des_thumb_mat(2,2));
  R_.getRPY(des_pose[3],des_pose[4],des_pose[5]);

  
  // Relaxed rigidity data:
  // [des_obj_pose(6x1), rigid_finger,fingers(4x1),position_offset(n_f*3),orientation_offset(n_f*3)]
  
  vector<int> fingers(begin(req.fingers), end(req.fingers));
  vector<double> fingers_d(fingers.begin(),fingers.end());
  vector<double> weights(begin(req.weights),end(req.weights));
  vector<double> r_fin;
  int r_f=req.rigid_finger;
  r_fin.resize(1,r_f);
  data_arr.resize(4+fingers.size()+1); 
  data_arr[0]=des_pose;
  data_arr[1]=weights;
  data_arr[2]=r_fin;
  data_arr[3]=fingers_d;
  
  Eigen::VectorXd thumb_pose;
  
  allegro_kdl.getFK(r_f,joints_0.segment(12,4),thumb_pose,true);
  vector<double> thumb_pose_arr(thumb_pose.data(),thumb_pose.data()+thumb_pose.size());
  
  data_arr[4+fingers.size()]=thumb_pose_arr;
  // compute offsets in thumb frame:
  // thumb matrix w_T_th=thumb_mat
  //compute inverse:
  Eigen::MatrixXd thumb_inv=thumb_mat.inverse();
  Eigen::VectorXd f_pose;
  Eigen::MatrixXd f_mat;
  vector<double> orient_offset;
  for(int i=0;i<fingers.size();++i)
  {
    int f=int(fingers_d[i]);
    // get joint angles for finger f:
    //get finger mat w_T_f
    allegro_kdl.getFK(f,joints_0.segment(f*4,4),f_mat);

    // Compute offset matrix:
    Eigen::MatrixXd offset_mat=thumb_inv*f_mat;
    data_mat.push_back(offset_mat);
    allegro_kdl.getFK(f,joints_0.segment(f*4,4),f_pose,true);
    
    Eigen::VectorXd pose_diff=thumb_pose-f_pose;
    vector<double> position_offset;
    position_offset.resize(pose_diff.size());
    
    pose_diff[0]=offset_mat(0,3);
    pose_diff[1]=offset_mat(1,3);
    pose_diff[2]=offset_mat(2,3);
    
    // Compute orientation offset:

    vector<double> thumb_orient(thumb_pose_arr.data()+3, thumb_pose_arr.data()+6);
    vector<double> f_orient(f_pose.data()+3, f_pose.data()+6);
    
    allegro_kdl.euler_diff(f_orient,thumb_orient,orient_offset);
    
    //orient_offset.resize(pose_diff.size());
    //VectorXd::Map(&orient_offset[0], pose_diff.size()) = pose_diff;
    pose_diff[3]=orient_offset[0];
    pose_diff[4]=orient_offset[1];
    pose_diff[5]=orient_offset[2];
    VectorXd::Map(&position_offset[0], pose_diff.size()) = pose_diff;

    // Computing position offset:
    data_arr[4+i]=position_offset;
    //data_arr[4+i+fingers.size()]=orient_offset;
    
  }

  
  //cerr<<"data size "<<data_arr.size()<<endl;
  vector<double> vel_limits;
  vel_limits.resize(16,req.weights[4]);
  int t_steps_=req.T;
  // Creating opt problem instance:
  cerr<<"creating opt problem"<<endl;
  vector<double> j0(joints_0.data(),joints_0.data()+joints_0.size());
  opt_problems::optProblem opt_problem(t_steps_,allegro_kdl,data_arr,data_mat,vel_limits,j0);

  problem prob{trajectory_optimization{opt_problem}};
  vector<double> tol(16*(t_steps_),1e-3);
 
  prob.set_c_tol(tol);
  pagmo::snopt7 sn_v(false,"/usr/local/lib/");
  sn_v.set_numeric_option("Major optimality tolerance",1e-3); 

  algorithm algo{sn_v};
  algo.set_verbosity(100);
  //algorithm algo{cmaes(500)};
  cerr<<"Creating pop"<<endl;
  population pop{prob,1u};
  
  vector<double> x_in(16*t_steps_,0.0);
  cerr<<"created pop"<<endl;
  // seed using final joint angles IK:

  for(int i=0;i<t_steps_;++i)
  {    
    for (int j=0;j<16;++j)
    {
      x_in[i*16+j]=joints_0[j];//+(final_joints[j]-joints_0[j])*(i+1)/(t_steps_);
    }
  }
  cerr<<"created pop"<<endl;
  // seed using previous joint trajectory:
  if(req.initial_jtraj.points.size()==t_steps_+1)
  {
    cerr<<"Using old trajectory for initialization"<<endl;
    for(int i=0;i<t_steps_;++i)
    {
      for (int j=0;j<16;++j)
      {
        x_in[i*16+j]=req.initial_jtraj.points[i+1].positions[j];
      }
    }
  }
  cerr<<"checking initial cost"<<endl;
  vector<double> f_in=prob.fitness(x_in);
  cerr<<"Initial fitness: "<<f_in[0]<<endl;
  if(req.initialize_opt==true)
  {
    pop.set_xf(0,x_in,f_in);
  }

  cerr<<"Solving"<<endl;  
  population pop_res = algo.evolve(pop);
  cerr<<"optimization completed!  Final cost: "<<pop_res.champion_f()[0]<<endl;



  vector<double> j_best=pop_res.champion_x();

  
  trajectory_msgs::JointTrajectory joint_traj;
  std::vector<std::string> joint_names={"index_joint_0","index_joint_1","index_joint_2","index_joint_3",
                                        "middle_joint_0","middle_joint_1","middle_joint_2","middle_joint_3",
                                        "ring_joint_0","ring_joint_1","ring_joint_2","ring_joint_3",
                                        "thumb_joint_0","thumb_joint_1","thumb_joint_2","thumb_joint_3"};
  joint_traj.joint_names=joint_names;    

  vector<double> j_arr;

  j_arr.resize(16);
  trajectory_msgs::JointTrajectoryPoint jt_pt;
  jt_pt.positions=j0;
  joint_traj.points.push_back(jt_pt);
  
  
  for (int i=0;i<j_best.size()/16;i++)
  { 
    for(int j=0;j<16;j++)
    {
      j_arr[j]=j_best[i*16+j];
    }  
    jt_pt.positions=j_arr;
    joint_traj.points.push_back(jt_pt);
  }

  res.joint_traj=joint_traj;

  // new object pose:
  Map<Eigen::VectorXd> th_j(&j_arr[12],4);
  allegro_kdl.getFK(3,th_j,thumb_mat);
  Eigen::MatrixXd  n_object_pose=thumb_mat*obj_offset;
  vector<double> n_pose;
  allegro_kdl.frame_to_pose_(n_object_pose,n_pose,false);
  /*
  // convert object frame to pose:
  tf::Matrix3x3 R_(n_object_pose(0,0), n_object_pose(0,1), n_object_pose(0,2),
                   n_object_pose(1,0), n_object_pose(1,1), n_object_pose(1,2),
                   n_object_pose(2,0), n_object_pose(2,1), n_object_pose(2,2));
  tf::Quaternion q_;
  R_.getRotation(q_);
  */
  geometry_msgs::PoseStamped n_obj_pose_;

  n_obj_pose_.pose.position.x=n_pose[0];
  n_obj_pose_.pose.position.y=n_pose[1];
  n_obj_pose_.pose.position.z=n_pose[2];
  n_obj_pose_.pose.orientation.x=n_pose[3];
  n_obj_pose_.pose.orientation.y=n_pose[4];
  n_obj_pose_.pose.orientation.z=n_pose[5];
  n_obj_pose_.pose.orientation.w=n_pose[6];
  res.n_obj_pose=n_obj_pose_;

  
  // compute object trajectory:
  geometry_msgs::PoseArray obj_pose_traj;
  obj_pose_traj.poses.resize((j_best.size()/16)+1);
  obj_pose_traj.header.frame_id="palm_link";
  // adding initial object pose:

  allegro_kdl.getFK(3,joints_0.segment(12,4),thumb_mat);
  n_object_pose=thumb_mat*obj_offset;
  allegro_kdl.frame_to_pose_(n_object_pose,n_pose,false);
  
  obj_pose_traj.poses[0].position.x=n_pose[0];
  obj_pose_traj.poses[0].position.y=n_pose[1];
  obj_pose_traj.poses[0].position.z=n_pose[2];
  obj_pose_traj.poses[0].orientation.x=n_pose[3];
  obj_pose_traj.poses[0].orientation.y=n_pose[4];
  obj_pose_traj.poses[0].orientation.z=n_pose[5];
  obj_pose_traj.poses[0].orientation.w=n_pose[6];

  Eigen::VectorXd thumb_joints;
  thumb_joints.resize(4);
  for (int i=0;i<j_best.size()/16;i++)
  {
   
    thumb_joints.setZero();
    for (int j=4*r_f;j<4*r_f+4;++j)
    {
      thumb_joints[j-4*r_f]=j_best[i*16+j];

    }

    allegro_kdl.getFK(3,thumb_joints,thumb_mat);
    n_object_pose=thumb_mat*obj_offset;
    allegro_kdl.frame_to_pose_(n_object_pose,n_pose,false);
  
    obj_pose_traj.poses[i+1].position.x=n_pose[0];
    obj_pose_traj.poses[i+1].position.y=n_pose[1];
    obj_pose_traj.poses[i+1].position.z=n_pose[2];
    obj_pose_traj.poses[i+1].orientation.x=n_pose[3];
    obj_pose_traj.poses[i+1].orientation.y=n_pose[4];
    obj_pose_traj.poses[i+1].orientation.z=n_pose[5];
    obj_pose_traj.poses[i+1].orientation.w=n_pose[6];
    
  }

  res.obj_traj=obj_pose_traj;

  // compute metrics:
  // RR cost:
  
  return true;
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"in_grasp_server");
  ros::NodeHandle n;
  ros::ServiceServer service=n.advertiseService("/in_hand/primitive/in_grasp_manipulation",in_grasp_call);
  
  ros::spin();// Loop rate?
}
