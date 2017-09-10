
// optimization:
#include <optimization_pkg/wrapper/pagmo_wrapper.hpp>

// ROS:
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include <Eigen/Geometry> 

// ROS service for in grasp manipulation:
#include "in_grasp_manipulation/InGraspManipulationSd.h"

using namespace pagmo;

collisionChecker c_check;
bool in_grasp_call(in_grasp_manipulation::InGraspManipulationSd::Request &req,in_grasp_manipulation::InGraspManipulationSd::Response &res)
{

  cerr<<"Calling optimization"<<endl;
  // Allegro URDF file:
  // TODO: ros::package segfaults, fix!
  string urdf_file="/home/bala/catkin_ws/src/in_grasp_manipulation/urdf/allegro.urdf";

  // Load allegro kdl object:
  vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<string> link_names={ "index_link_1","index_link_2","index_tip",
                              "middle_link_1","middle_link_2","middle_tip",
                              "ring_link_1","ring_link_2","ring_tip",
                              "thumb_link_2","thumb_link_3","thumb_tip"};


  vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics

  manipulator_kdl::robotKDL allegro_kdl(urdf_file,base_names,ee_names,link_names,g_vec);
  allegro_kdl.urdfParser();
  

  // Getting parameters from service request:
  vector<double> joints_0;
  joints_0.resize(16);
  for(int i=0;i<req.init_joints.position.size();++i)
  {
    joints_0[i]=req.init_joints.position[i];
  }

  
  vector<vector<double>> data_arr;

  // Get initial object to thumb frame:
  tf::Pose init_pose;
  tf::poseMsgToTF(req.object_pose.pose,init_pose);
  // init_pose->4x4
  tf::Quaternion q(req.object_pose.pose.orientation.x, req.object_pose.pose.orientation.y, req.object_pose.pose.orientation.z, req.object_pose.pose.orientation.w);
  tf::Matrix3x3 R(q);
  Eigen::MatrixXd obj_mat(4,4);
  obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    obj_mat(i,0)=R[i].x();
    obj_mat(i,1)=R[i].y();
    obj_mat(i,2)=R[i].z();
  }
  obj_mat(0,3)=req.object_pose.pose.position.x;
  obj_mat(1,3)=req.object_pose.pose.position.y;
  obj_mat(2,3)=req.object_pose.pose.position.z;

  // Update object pose in collision checker:
  vector<double> obj_pose_q;
  obj_pose_q.resize(7);
  obj_pose_q[0]=req.object_pose.pose.position.x;
  obj_pose_q[1]=req.object_pose.pose.position.y;
  obj_pose_q[2]=req.object_pose.pose.position.z;
  obj_pose_q[3]=req.object_pose.pose.orientation.x;
  obj_pose_q[4]=req.object_pose.pose.orientation.y;
  obj_pose_q[5]=req.object_pose.pose.orientation.z;
  obj_pose_q[6]=req.object_pose.pose.orientation.w;
  c_check.update_env_pose(obj_pose_q);

  
  vector<double> thumb_joints;
  thumb_joints.push_back(joints_0[12]);
  thumb_joints.push_back(joints_0[13]);
  thumb_joints.push_back(joints_0[14]);
  thumb_joints.push_back(joints_0[15]);

  Eigen::MatrixXd thumb_mat=allegro_kdl.getFKEigen(3,thumb_joints);
  
  Eigen::MatrixXd obj_offset=thumb_mat.inverse()*obj_mat;



  // We do not have a desired object pose, we only have contact points:

  // Getting contact points and storiung in data_arr
  vector<vector<double>> d_cpts;
  for(int i=0;i<req.des_contact_pts.size();i++)
  {
    vector<double> pt;
    pt.push_back(req.des_contact_pts[i].point.x);
    pt.push_back(req.des_contact_pts[i].point.y);
    pt.push_back(req.des_contact_pts[i].point.z);
    d_cpts.push_back(pt);
  }
  
  
  vector<int> fingers(begin(req.f_index), end(req.f_index));
  vector<double> fingers_d(fingers.begin(),fingers.end());
  vector<double> weights(begin(req.weights),end(req.weights));
  vector<double> r_fin;
  int r_f=3;//req.rigid_finger;
 
  r_fin.resize(1,r_f);

  
  vector<double> des_pose;
  des_pose.resize(6);
  des_pose[0]=req.desired_object_pose.pose.position.x;
  des_pose[1]=req.desired_object_pose.pose.position.y;
  des_pose[2]=req.desired_object_pose.pose.position.z;

  // Get euler xyz from rotation matrix:
  tf::Quaternion q_1(req.desired_object_pose.pose.orientation.x, req.desired_object_pose.pose.orientation.y, req.desired_object_pose.pose.orientation.z, req.desired_object_pose.pose.orientation.w);
  tf::Matrix3x3 R_1(q_1);
  R_1.getRPY(des_pose[3],des_pose[4],des_pose[5]);

  
  data_arr.resize(4+fingers.size()+fingers.size()+d_cpts.size()+1); 
  data_arr[0]=obj_pose_q;
  data_arr[4+fingers.size()+fingers.size()+d_cpts.size()]=des_pose;
  data_arr[1]=weights;

  data_arr[2]=r_fin;
  data_arr[3]=fingers_d;
  for(int i=0;i<d_cpts.size();++i)
  {
    data_arr[4+fingers.size()+fingers.size()+i]=d_cpts[i];
  }

  
  vector<double> thumb_pose_arr=allegro_kdl.getFK(r_f,thumb_joints,true);
  Map<Eigen::VectorXd> thumb_pose(&thumb_pose_arr[0],thumb_pose_arr.size());;
  for(int i=0;i<fingers.size();++i)
  {
    int f=int(fingers_d[i]);
    vector<double> f_joints(&joints_0[f*4],&joints_0[f*4+4]);
    //cerr<<f_joints.size()<<endl;
    vector<double> f_pose_arr=allegro_kdl.getFK(f,f_joints,true);
    Map<Eigen::VectorXd> f_pose(&f_pose_arr[0],f_pose_arr.size());
    Eigen::VectorXd pose_diff=thumb_pose-f_pose;
    vector<double> position_offset;
    position_offset.resize(pose_diff.size());
    VectorXd::Map(&position_offset[0], pose_diff.size()) = pose_diff;

    vector<double> orient_offset;
    orient_offset.resize(pose_diff.size());
    VectorXd::Map(&orient_offset[0], pose_diff.size()) = pose_diff;

    // Computing position offset:
    data_arr[4+i]=position_offset;
    data_arr[4+i+fingers.size()]=orient_offset;
  }


  //cerr<<"data size "<<data_arr.size()<<endl;
  vector<double> vel_limits;
  vel_limits.resize(0,100.15);
  //cerr<<"calling pagmo"<<endl;
  int t_steps=1;
  // Creating opt problem instance:
  //collisionChecker c_check_=c_check;
  allegro_kdl.f_dof=4;
  opt_problems::optProblem opt_problem(t_steps, allegro_kdl, c_check, data_arr,vel_limits,joints_0);
  //opt_problems::optProblem opt_problem(1);
  //cerr<<"setup optimization"<<endl;
  problem prob{trajectory_optimization{opt_problem}};
  vector<double> tol(1,1e-3);
 
  prob.set_c_tol(tol);
  
  algorithm algo{snopt7(false,"/usr/local/lib/")};
  //algorithm algo{cmaes(500)};
  population pop{prob,10u};
  // Give initial seed:
  //vector<double> x_in=joints_0;
  //vector<double> f_in=prob.fitness(x_in);
  //pop.set_xf(0,x_in,f_in);

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
  for (int i=0;i<j_best.size()/16;i++)
  { 
    trajectory_msgs::JointTrajectoryPoint jt_pt;
    for(int j=0;j<16;j++)
    {
      j_arr[j]=j_best[i*16+j];
    }
    jt_pt.positions=j_arr;
    joint_traj.points.push_back(jt_pt);
    
  }
  sensor_msgs::JointState js;
  js.position=j_arr;
  js.name=joint_names;
  res.n_js=js;
  res.joint_traj=joint_traj;

  // Update object pose:
  // Get initial object to thumb frame:
  // Get fk of current thumb joints:
  thumb_joints.clear();
  thumb_joints.push_back(j_arr[12]);
  thumb_joints.push_back(j_arr[13]);
  thumb_joints.push_back(j_arr[14]);
  thumb_joints.push_back(j_arr[15]);
  thumb_mat=allegro_kdl.getFKEigen(3,thumb_joints);
  Eigen::MatrixXd  n_object_pose=thumb_mat*obj_offset;
  vector<double> n_pose=allegro_kdl.frame_to_pose_(n_object_pose,false);
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
  return true;
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"in_grasp_server");
  ros::NodeHandle n;
  c_check.min_env_size=3;
  
  // Initialize collision checking:
  
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
                          "thumb_link_2.ply","thumb_link_3.ply","thumb_tip.ply",
                          "reachable_cvx_index.ply","reachable_cvx_middle.ply", "reachable_cvx_ring.ply",
                          "reachable_cvx_thumb.ply"};
  for(int i=0;i<r_files.size();++i)
  {
    r_files[i].insert(0,mesh_location);
  }
  c_check.initialize_robot_state(r_files.size(),r_files);

  string obj_file="/home/bala/object_models/banana/google_16k/vertices.ply";

  //string obj_file="/home/bala/catkin_ws/src/in_hand_manipulation/meshes/banana.ply";
  vector<double> pose;
  //c_check.initialize_robot_state(1,f_names);
  c_check.add_obj_cloud(obj_file,pose);



  
  ros::ServiceServer service=n.advertiseService("/in_hand/primitive/in_grasp_manipulation",in_grasp_call);
  ros::spin();// Loop rate?
}
