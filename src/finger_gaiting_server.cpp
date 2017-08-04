
// optimization:
#include <optimization_pkg/wrapper/pagmo_wrapper.hpp>
// ROS:
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include <Eigen/Geometry> 

// ROS service for finger gaiting
#include "in_hand_manipulation/PlanFingerGait.h"

//#include <in_grasp_manipulation/InGraspManipulation.h>
using namespace pagmo;
collisionChecker c_check;

bool finger_gait_call(in_hand_manipulation::PlanFingerGait::Request &req, in_hand_manipulation::PlanFingerGait::Response &res)
{
  // Get data:
  vector<double> finger_des_pt;
  finger_des_pt.resize(3);
  finger_des_pt[0]=req.desired_finger_pose.position.x;
  finger_des_pt[1]=req.desired_finger_pose.position.y;
  finger_des_pt[2]=req.desired_finger_pose.position.z;

  vector<double> finger;
  finger.push_back(req.finger);
  cerr<<"Got finger"<<endl;
  


  // Get joints for finger:
  vector<double> f_joints;
  f_joints.resize(4);// dof per finger
  f_joints[0]=finger[0]*4+0;
  f_joints[1]=finger[0]*4+1;
  f_joints[2]=finger[0]*4+2;
  f_joints[3]=finger[0]*4+3;

    // Initial joint angles:
  vector<double> init_joints;
  init_joints.push_back(req.init_joints.position[f_joints[0]]);
  init_joints.push_back(req.init_joints.position[f_joints[1]]);
  init_joints.push_back(req.init_joints.position[f_joints[2]]);
  init_joints.push_back(req.init_joints.position[f_joints[3]]);

  // Setup data_arr for optimzation:
  // data_arr=[finger_des_pt,finger,f_joints]
  vector<vector<double>> data_arr;
  data_arr.resize(3);
  data_arr[0]=finger_des_pt;
  data_arr[1]=finger;
  data_arr[2]=f_joints;
  
  // Build allegro kdl tree:
  string urdf_file="/home/bala/catkin_ws/src/in_grasp_manipulation/urdf/allegro.urdf";

  // Load allegro kdl object:
  vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics, lets assume we are in space.

  manipulator_kdl::robotKDL allegro_kdl(urdf_file,base_names,ee_names,g_vec);
  allegro_kdl.urdfParser();
  
  // TODO:Setup collision checking

  // Setup optimization:
  int t_steps=6;
  allegro_kdl.dof=4;
  vector<double> vel_limits;
  vel_limits.resize(4,0.15);
  cerr<<"creating opt problem"<<endl;
  //opt_problems::optProblem opt_problem(1);
  opt_problems::optProblem opt_problem_(t_steps,allegro_kdl,data_arr,vel_limits,init_joints);
  cerr<<opt_problem_.m_dim<<endl;

  cerr<<"creating pagmo problem"<<endl;
  problem prob{trajectory_optimization{opt_problem_}};
  cerr<<"Created pagmo problem"<<endl;
  vector<double> tol(t_steps*4,1e-3);
  cerr<<"Setting tolerances"<<endl;
  prob.set_c_tol(tol);
  cerr<<"Linking to snop"<<endl;
  algorithm algo{snopt7(false,"/usr/local/lib/")};
  //algorithm algo{cmaes(500)};
  cerr<<"creating population"<<endl;
  population pop{prob,1u};
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
  j_arr=req.init_joints.position;
  for (int i=0;i<t_steps;i++)
  { 
    trajectory_msgs::JointTrajectoryPoint jt_pt;
    for(int j=0;j<4;j++)
    {

      j_arr[f_joints[j]]=j_best[i*4+j];
    }    
    jt_pt.positions=j_arr;
    joint_traj.points.push_back(jt_pt);

  }
  res.joint_traj=joint_traj;

  return true;
  
}



int main(int argc,char** argv)
{
  //finger_gait_call();
  ros::init(argc,argv,"finger_gait_server");
  ros::NodeHandle n;

  // Setup collision checker:
  // Add object pointcloud:
  c_check.add_obj_cloud();
  // Publish object pointcloud:
  ros::Publisher pub=n.advertise<pcl::PointCloud<PointXYZRGB>>("/hacd/env_objects",1);

  // Add robot meshes:
  ros::ServiceServer service=n.advertiseService("/in_hand/primitive/finger_gaiting",finger_gait_call);
  ros::spin();// Loop rate?
}
