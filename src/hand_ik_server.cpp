
// optimization:
#include <optimization_pkg/wrapper/pagmo_wrapper.hpp>
// ROS:
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include <Eigen/Geometry> 

// ROS service for finger gaiting
#include "in_hand_manipulation/GetHandIK.h"

using namespace pagmo;

bool hand_ik_call(in_hand_manipulation::GetHandIK::Request &req,
                  in_hand_manipulation::GetHandIK::Response &res)
{
  // Get data from service request:
 // Build allegro kdl tree:
  string urdf_file="/home/bala/catkin_ws/src/in_grasp_manipulation/urdf/allegro.urdf";

  // Load allegro kdl object:
  vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics, lets assume we are in space.

  manipulator_kdl::robotKDL allegro_kdl(urdf_file,base_names,ee_names,g_vec);
  allegro_kdl.urdfParser();

  vector<double> joints_0;
  joints_0.resize(16);
  for(int i=0;i<req.init_joint_state.position.size();++i)
  {
    joints_0[i]=req.init_joint_state.position[i];
  }

  
  vector<vector<double>> data_arr;

  // Get initial finger poses:
  for(int i=0;i<4;++i)
  {
    vector<double> f_des_pose;
    f_des_pose.resize(3);
    f_des_pose[0]=req.desired_poses[i].position.x;
    f_des_pose[1]=req.desired_poses[i].position.y;
    f_des_pose[2]=req.desired_poses[i].position.z;
    data_arr.push_back(f_des_pose);
  }
  // Get f_index:
  vector<double> f_index;
  f_index.resize(4,0);
  for(int j=0;j<req.f_index.size();++j)
  {
    int loc=req.f_index[j].data;
    f_index[loc]=1.0;
  }
  data_arr.push_back(f_index);

  vector<double> vel_limits;
  vel_limits.resize(0,0.);
  // Creating opt problem instance:
  opt_problems::optProblem opt_problem(1,allegro_kdl,data_arr,vel_limits,joints_0);

  problem prob{trajectory_optimization{opt_problem}};
  vector<double> tol(0,1e-3);
 
  prob.set_c_tol(tol);
  
  algorithm algo{snopt7(false,"/usr/local/lib/")};
  //algorithm algo{cmaes(500)};
  population pop{prob,10u};
  cerr<<"Solving"<<endl;  
  population pop_res = algo.evolve(pop);
  cerr<<"optimization completed!  Final cost: "<<pop_res.champion_f()[0]<<endl;



  vector<double> j_best=pop_res.champion_x();

  sensor_msgs::JointState goal_state;
  goal_state=req.init_joint_state;
  goal_state.position=j_best;

  res.joint_state=goal_state;
  return true;   
}



int main(int argc,char** argv)
{
  ros::init(argc,argv,"hand_ik_server");
  ros::NodeHandle n;
  // Add robot meshes:
  ros::ServiceServer service=n.advertiseService("/in_hand/ik",hand_ik_call);
  ros::spin();// Loop rate?
}
