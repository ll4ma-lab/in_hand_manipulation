
// optimization:
#include <optimization_pkg/wrapper/pagmo_wrapper.hpp>
// ROS:
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>

#include <Eigen/Geometry> 

// ROS service for finger gaiting
#include "in_hand_manipulation/GetGrasp.h"

// quaternion to euler:
#include <tf/transform_datatypes.h>

//#include <in_grasp_manipulation/InGraspManipulation.h>
using namespace pagmo;
collisionChecker c_check;

bool grasp_call(in_hand_manipulation::GetGrasp::Request &req, in_hand_manipulation::GetGrasp::Response &res)
{
  // Get data:
  vector<double> obj_pose;
  vector<double> obj_pose_q;
  obj_pose.resize(6);
  obj_pose_q.resize(7);
  
  tf::Quaternion q(req.obj_pose.pose.orientation.x, req.obj_pose.pose.orientation.y,
                   req.obj_pose.pose.orientation.z, req.obj_pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(obj_pose[3], obj_pose[4], obj_pose[5]);
  obj_pose[0]=req.obj_pose.pose.position.x;
  obj_pose[1]=req.obj_pose.pose.position.y;
  obj_pose[2]=req.obj_pose.pose.position.z;
  obj_pose_q[0]=req.obj_pose.pose.position.x;
  obj_pose_q[1]=req.obj_pose.pose.position.y;
  obj_pose_q[2]=req.obj_pose.pose.position.z;
  obj_pose_q[3]=req.obj_pose.pose.orientation.x;
  obj_pose_q[4]=req.obj_pose.pose.orientation.y;
  obj_pose_q[5]=req.obj_pose.pose.orientation.z;
  obj_pose_q[6]=req.obj_pose.pose.orientation.w;
  c_check.update_env_pose(obj_pose_q);
  vector<double> finger(std::begin(req.f_index),std::end(req.f_index));


  

  
  vector<double> obj_pose_low_bounds={0.0,0.0,0.0,0.0,0.0,0.0};
  vector<double> obj_pose_up_bounds={0.0,0.0,0.0,0.0,0.0,0.0};
  // Setup data_arr for optimzation:
  // data_arr=[finger_des_pt,finger,f_joints]
  vector<vector<double>> data_arr;
  data_arr.resize(5);
  data_arr[0]=obj_pose;
  data_arr[1]=finger;
  data_arr[2]=obj_pose_low_bounds;
  data_arr[3]=obj_pose_up_bounds;
  vector<double> cpt_dist;
  cpt_dist.push_back(-0.005);
  data_arr[4]=cpt_dist;
  
  // Build allegro kdl tree:
  string urdf_file="/home/bala/catkin_ws/src/in_grasp_manipulation/urdf/allegro.urdf";

  // Load allegro kdl object:
  vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<string> link_names={ "index_link_1","index_link_2","index_tip",
                             "middle_link_1","middle_link_2","middle_tip",
                             "ring_link_1","ring_link_2","ring_tip",
                             "thumb_link_2","thumb_link_3","thumb_tip"};


  vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics, lets assume we are in space.

  manipulator_kdl::robotKDL allegro_kdl(urdf_file,base_names,ee_names,link_names,g_vec);
  allegro_kdl.urdfParser();
  
  // TODO:Setup collision checking

  // Setup optimization:
  vector<double> init_joints;
  init_joints.resize(16,0.5);
  allegro_kdl.dof=16;
  allegro_kdl.f_dof=4;
  vector<double> vel_limits;
  vel_limits.resize(2,0.15);
  cerr<<"creating opt problem"<<endl;
  int t_steps=1;
  opt_problems::optProblem opt_problem_(t_steps,allegro_kdl,c_check,data_arr,vel_limits,init_joints);
  cerr<<opt_problem_.m_dim<<endl;

  cerr<<"creating pagmo problem"<<endl;
  problem prob{trajectory_optimization{opt_problem_}};
  cerr<<"Created pagmo problem"<<endl;
  vector<double> tol(8,1e-3);// for constraints
  
  cerr<<"Setting tolerances"<<endl;
  prob.set_c_tol(tol);
  cerr<<"Linking to snop"<<endl;
  algorithm algo{snopt7(false,"/usr/local/lib/")};
  //algorithm algo{cmaes(500)};
  cerr<<"creating population"<<endl;
  population pop{prob,1u};
  vector<double> x0;
  x0.resize(22,0.0);
  vector<double> x_in=x0;
  vector<double> f_in=prob.fitness(x_in);
  pop.set_xf(0,x_in,f_in);

  cerr<<"Solving"<<endl;  
  population pop_res = algo.evolve(pop);
  cerr<<"optimization completed!  Final cost: "<<pop_res.champion_f()[0]<<endl;



  vector<double> j_best=pop_res.champion_x();
  sensor_msgs::JointState j_state;


  //trajectory_msgs::JointTrajectory joint_traj;
  std::vector<std::string> joint_names={"index_joint_0","index_joint_1","index_joint_2","index_joint_3",
                                        "middle_joint_0","middle_joint_1","middle_joint_2","middle_joint_3",
                                        "ring_joint_0","ring_joint_1","ring_joint_2","ring_joint_3",
                                        "thumb_joint_0","thumb_joint_1","thumb_joint_2","thumb_joint_3"};
  j_state.name=joint_names;    

  vector<double> j_arr;

  j_arr.resize(16);
  for (int i=0;i<16;i++)
  { 
      j_arr[i]=j_best[i];
  }
  j_state.position=j_arr;
  res.joint_state=j_state;

  // object pose:
  
  return true;
  
}



int main(int argc,char** argv)
{
  //finger_gait_call();
  ros::init(argc,argv,"finger_gait_server");
  ros::NodeHandle n;
  c_check.min_env_size=5;

  // Setup collision checker:
  // Add object pointcloud:
  tf::TransformListener tf_l;
  c_check.tf_listen=&tf_l;

  ros::Publisher pub=n.advertise<pcl::PointCloud<PointXYZRGB>>("/hacd/env_objects",1);
  c_check.pub_=pub;
  // load robot links:
    
  string mesh_location = "/home/bala/catkin_ws/src/urlg_robots_description";//

  //std::string mesh_location=ros::package::getPath("urlg_robots_description");
  //cerr<<mesh_location.c_str()<<endl;
  mesh_location.append("/meshes/allegro/convex_hull/vertices/");
  
  // robot mesh file names:
  vector<string> r_files={"palm_link.ply",
                          "index_link_1.ply","index_link_2.ply","index_tip.ply",
                          "index_link_1.ply","index_link_2.ply","index_tip.ply",
                          "index_link_1.ply","index_link_2.ply","index_tip.ply",
                          "thumb_link_2.ply","thumb_link_3.ply","thumb_tip.ply"};

  for(int i=0;i<r_files.size();++i)
  {
    r_files[i].insert(0,mesh_location);
  }
  c_check.initialize_robot_state(r_files.size(),r_files);
  


  string obj_file="/home/bala/object_models/banana/google_16k/vertices.ply";
  vector<double> pose;
  
  //c_check.initialize_robot_state(1,f_names);
  c_check.add_obj_cloud(obj_file,pose);

  
  // Publish object pointcloud:

  // Add robot meshes:
  ros::ServiceServer service=n.advertiseService("/in_hand/gen_grasp",grasp_call);
  ros::spin();// Loop rate?
}
