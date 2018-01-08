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

  cerr<<"Creating fingertip poses:"<<endl;
  Map<Eigen::VectorXd> joints_0(&req.init_joints.position[0],req.init_joints.position.size());

  Eigen::VectorXd initial_pose(6);
  // getting initial object pose:
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
  
  initial_pose[0]=req.object_pose.position.x;
  initial_pose[1]=req.object_pose.position.y;
  initial_pose[2]=req.object_pose.position.z;
  R.getRPY(initial_pose[3],initial_pose[4],initial_pose[5]);
  // desired object pose mat:

  tf::Quaternion q_des(req.desired_object_pose.orientation.x,req.desired_object_pose.orientation.y,req.desired_object_pose.orientation.z,req.desired_object_pose.orientation.w);
  tf::Matrix3x3 R_des(q_des);
  Eigen::VectorXd des_obj_pose(6);
  Eigen::MatrixXd des_obj_mat(4,4);
  des_obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    des_obj_mat(i,0)=R_des[i][0];
    des_obj_mat(i,1)=R_des[i][1];
    des_obj_mat(i,2)=R_des[i][2];
  }
  des_obj_mat(0,3)=req.desired_object_pose.position.x;
  des_obj_mat(1,3)=req.desired_object_pose.position.y;
  des_obj_mat(2,3)=req.desired_object_pose.position.z;
  des_obj_pose[0]=req.desired_object_pose.position.x;
  des_obj_pose[1]=req.desired_object_pose.position.y;
  des_obj_pose[2]=req.desired_object_pose.position.z;
  R_des.getRPY(des_obj_pose[3], des_obj_pose[4], des_obj_pose[5]);
  // linearly interpolate object poses:
  cerr<<"linearly interpolating poses"<<endl;
  vector<Eigen::MatrixXd> lin_obj_pose;
  lin_obj_pose.resize(req.T);
  
  for(int i=0;i<req.T;++i)
  {
    //
    Eigen::VectorXd c_pose=initial_pose+(des_obj_pose-initial_pose)*double(i+1)/double(req.T);
    //cerr<<c_pose<<endl;
    allegro_kdl.pose_to_frame(c_pose,lin_obj_pose[i]);
    //cerr<<"stored frame"<<endl;
  }
  // Assuming the initial object pose is the centre of rotation of the object:
  // Build tf frames for fingertips to initial object pose
  cerr<<"computing fingertip poses"<<endl;
  vector<vector<Eigen::VectorXd>> f_poses;
  f_poses.resize(req.fingers.size());
  for(int i=0;i<req.fingers.size();++i)
  {
    int f=req.fingers[i];
    Eigen::MatrixXd p_T_f;
    allegro_kdl.getFK(f, joints_0.segment(f*4,4), p_T_f);
    // o_T_f=obj_mat.inverse()*p_T_f
    Eigen::MatrixXd o_T_f=obj_mat.inverse()*p_T_f;

    // loop over timesteps:
    for(int j=0;j<req.T;++j)
    {
      // desired fingertip pose:  
      Eigen::MatrixXd d_T_f=lin_obj_pose[j]*o_T_f;
      // converting to pose:
      vector<double> des_f_pose;
      allegro_kdl.frame_to_pose_(d_T_f,des_f_pose);
    // convert to eigen
      Map<Eigen::VectorXd> f_pose(&des_f_pose[0],des_f_pose.size());
      f_poses[i].push_back(f_pose);
    }
    }
  
 


  
  
  vector<int> fingers(begin(req.fingers), end(req.fingers));
  vector<double> weights(begin(req.weights),end(req.weights));
 
  vector<double> vel_limits;
  vel_limits.resize(16,req.weights[2]);
  int t_steps_=req.T;
  // Creating opt problem instance:
  cerr<<"creating opt problem"<<endl;
  vector<double> j0(joints_0.data(),joints_0.data()+joints_0.size());
  opt_problems::optProblem opt_problem(t_steps_,allegro_kdl,weights,f_poses,vel_limits,j0,fingers);

  problem prob{trajectory_optimization{opt_problem}};
  vector<double> tol(16*(t_steps_),1e-3);
 
  prob.set_c_tol(tol);
  pagmo::snopt7 sn_v(false,"/usr/local/lib/");
  //sn_v.set_numeric_option("Major optimality tolerance",1e-3); 

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
      x_in[i*16+j]=joints_0[j];
    }
  }
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


  // compute metrics:
  // RR cost:
  
  return true;
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"in_grasp_server");
  ros::NodeHandle n;
  ros::ServiceServer service=n.advertiseService("/in_hand/primitive/in_grasp_rolling",in_grasp_call);
  
  ros::spin();// Loop rate?
}
