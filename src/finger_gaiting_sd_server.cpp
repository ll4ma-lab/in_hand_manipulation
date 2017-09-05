
// optimization:
#include <optimization_pkg/wrapper/pagmo_wrapper.hpp>
// ROS:
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include <Eigen/Geometry> 

// ROS service for finger gaiting
//#include "in_hand_manipulation/PlanFingerGait.h"
#include <grasp_gaits/RelocateFingerKin.h>
#include <sensor_msgs/JointState.h>
//#include <in_grasp_manipulation/InGraspManipulation.h>
using namespace pagmo;
collisionChecker c_check;

bool finger_gait_call(grasp_gaits::RelocateFingerKin::Request &req,  grasp_gaits::RelocateFingerKin::Response &res)
{
  // Get data:
  vector<double> obj_pose_q;
  obj_pose_q.resize(7);
  obj_pose_q[0]=req.obj_pose.pose.position.x;
  obj_pose_q[1]=req.obj_pose.pose.position.y;
  obj_pose_q[2]=req.obj_pose.pose.position.z;
  obj_pose_q[3]=req.obj_pose.pose.orientation.x;
  obj_pose_q[4]=req.obj_pose.pose.orientation.y;
  obj_pose_q[5]=req.obj_pose.pose.orientation.z;
  obj_pose_q[6]=req.obj_pose.pose.orientation.w;
  c_check.update_env_pose(obj_pose_q);



  
  vector<double> finger_des_pt;
  finger_des_pt.resize(3);
  finger_des_pt[0]=req.des_contact_pt.point.x;
  finger_des_pt[1]=req.des_contact_pt.point.y;
  finger_des_pt[2]=req.des_contact_pt.point.z;

  vector<double> finger;
  finger.push_back(req.f_idx);
  cerr<<"Got finger: "<<finger[0]<<endl;
  


  // Get joints for finger:
  vector<double> f_joints;
  f_joints.resize(4);// dof per finger
  f_joints[0]=finger[0]*4+0;
  f_joints[1]=finger[0]*4+1;
  f_joints[2]=finger[0]*4+2;
  f_joints[3]=finger[0]*4+3;

    // Initial joint angles:
  vector<double> init_joints;
  init_joints.push_back(req.js.position[f_joints[0]]);
  init_joints.push_back(req.js.position[f_joints[1]]);
  init_joints.push_back(req.js.position[f_joints[2]]);
  init_joints.push_back(req.js.position[f_joints[3]]);

  // Setup data_arr for optimzation:
  // data_arr=[finger_des_pt,finger,f_joints]
  vector<vector<double>> data_arr;
  data_arr.resize(5);
  data_arr[0]=finger_des_pt;
  data_arr[1]=finger;
  data_arr[2]=f_joints;

  vector<double> cpt_dist;
  cpt_dist.push_back(-0.1);
  data_arr[4]=cpt_dist;

  
  // Build allegro kdl tree:
  string urdf_file="/home/bala/catkin_ws/src/in_grasp_manipulation/urdf/allegro.urdf";

  // Load allegro kdl object:
  vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics, lets assume we are in space.
  cerr<<"creating robot kdl"<<endl;
  manipulator_kdl::robotKDL allegro_kdl(urdf_file,base_names,ee_names,g_vec);
  allegro_kdl.urdfParser();
  
  // TODO:Setup collision checking

  // Setup optimization:
  int t_steps=2;
  allegro_kdl.dof=4;
  vector<double> vel_limits;
  vel_limits.resize(4,100.15);
  cerr<<"creating opt problem"<<endl;
  //opt_problems::optProblem opt_problem(1);
  opt_problems::optProblem opt_problem_(t_steps,allegro_kdl,c_check,data_arr,vel_limits,init_joints);
  //cerr<<opt_problem_.m_dim<<endl;

  //cerr<<"creating pagmo problem"<<endl;
  problem prob{trajectory_optimization{opt_problem_}};
  //cerr<<"Created pagmo problem"<<endl;
  vector<double> tol(t_steps*4+1,1e-3);
  //cerr<<"Setting tolerances"<<endl;
  prob.set_c_tol(tol);
  //cerr<<"Linking to snop"<<endl;
  algorithm algo{snopt7(false,"/usr/local/lib/")};
  //algorithm algo{cmaes(500)};
  //cerr<<"creating population"<<endl;
  population pop{prob,1u};
  cerr<<"Solving"<<endl;  
  population pop_res = algo.evolve(pop);
  cerr<<"optimization completed!  Final cost: "<<pop_res.champion_f()[0]<<endl;



  vector<double> j_best=pop_res.champion_x();

  
  //trajectory_msgs::JointTrajectory joint_traj;
  std::vector<std::string> joint_names={"index_joint_0","index_joint_1","index_joint_2","index_joint_3",
                                        "middle_joint_0","middle_joint_1","middle_joint_2","middle_joint_3",
                                        "ring_joint_0","ring_joint_1","ring_joint_2","ring_joint_3",
                                        "thumb_joint_0","thumb_joint_1","thumb_joint_2","thumb_joint_3"};
  //joint_traj.joint_names=joint_names;    

  vector<double> j_arr;

  j_arr.resize(16);
  j_arr=req.js.position;
  sensor_msgs::JointState js;
  //float[16] j_arr_;
  js.name=joint_names;
  for (int i=0;i<t_steps;i++)
  { 
    //trajectory_msgs::JointTrajectoryPoint jt_pt;
    for(int j=0;j<4;j++)
    {
      j_arr[f_joints[j]]=j_best[i*4+j];
    }    
   
  }
  js.position=j_arr;
  res.n_js=js;
  //res.joint_traj=joint_traj;

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

  //string obj_file="/home/bala/catkin_ws/src/urlg_robots_description/meshes/lbr4/convex_hull/vertices/link_1.ply";
  //vector<string> f_names;
  //f_names.push_back(obj_file);
  string obj_file="/home/bala/object_models/banana/google_16k/vertices.ply";

  //string obj_file="/home/bala/catkin_ws/src/in_hand_manipulation/meshes/banana.ply";
  vector<double> pose;
  //c_check.initialize_robot_state(1,f_names);
  c_check.add_obj_cloud(obj_file,pose);
  
  // Publish object pointcloud:

  // Add robot meshes:
  ros::ServiceServer service=n.advertiseService("regrasp_planner/relocate_finger",finger_gait_call);
  ros::spin();// Loop rate?
}
