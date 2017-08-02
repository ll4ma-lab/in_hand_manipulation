
// optimization:
#include <optimization_pkg/wrapper/pagmo_wrapper.hpp>
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
  // TODO: ros::package segfaults, fix!
  string urdf_file="/home/bala/catkin_ws/src/in_grasp_manipulation/urdf/allegro.urdf";

  // Load allegro kdl object:
  vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics

  manipulator_kdl::robotKDL allegro_kdl(urdf_file,base_names,ee_names,g_vec);
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
  tf::poseMsgToTF(req.object_pose,init_pose);
  // init_pose->4x4
  tf::Quaternion q(req.object_pose.orientation.x,req.object_pose.orientation.y,req.object_pose.orientation.z,req.object_pose.orientation.w);
  tf::Matrix3x3 R(q);
  Eigen::MatrixXd obj_mat(4,4);
  obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    obj_mat(i,0)=R[i][0];
    obj_mat(i,1)=R[i].y();
    obj_mat(i,2)=R[i].z();
  }
  obj_mat(3,0)=req.object_pose.position.x;
  obj_mat(3,1)=req.object_pose.position.y;
  obj_mat(3,2)=req.object_pose.position.z;

  
  vector<double> thumb_joints;
  thumb_joints.push_back(joints_0[12]);
  thumb_joints.push_back(joints_0[13]);
  thumb_joints.push_back(joints_0[14]);
  thumb_joints.push_back(joints_0[15]);
  Eigen::MatrixXd thumb_mat=allegro_kdl.getFKEigen(3,thumb_joints);
  
  Eigen::MatrixXd obj_offset=thumb_mat.inverse()*obj_mat;
  
  tf::Pose des_pose_tf;
  tf::poseMsgToTF(req.desired_object_pose,des_pose_tf);
  // convert desired object pose to thumb pose:
  R=des_pose_tf.getBasis();
  
  Eigen::MatrixXd des_obj_mat(4,4);
  des_obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    des_obj_mat(i,0)=R[i].x();
    des_obj_mat(i,1)=R[i].y();
    des_obj_mat(i,2)=R[i].z();
  }
  des_obj_mat(3,0)=req.desired_object_pose.position.x;
  des_obj_mat(3,1)=req.desired_object_pose.position.y;
  des_obj_mat(3,2)=req.desired_object_pose.position.z;
  Eigen::MatrixXd des_thumb_mat=des_obj_mat*obj_offset.inverse();

  // Get des pose from mat:
  //thumb_joints[0]=0.8;
  //vector<double> des_pose_thumb=allegro_kdl.getFK(3,thumb_joints,true);
  vector<double> des_pose;
  des_pose.resize(6);
  des_pose[0]=des_thumb_mat(3,0);
  des_pose[1]=des_thumb_mat(3,1);
  des_pose[2]=des_thumb_mat(3,2);
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
  data_arr.resize(4+fingers.size()+fingers.size()); 
  data_arr[0]=des_pose;
  data_arr[1]=weights;
  data_arr[2]=r_fin;
  data_arr[3]=fingers_d;
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
  vel_limits.resize(16,0.15);
  // Creating opt problem instance:
  opt_problems::optProblem opt_problem(10,allegro_kdl,data_arr,vel_limits,joints_0);

  problem prob{trajectory_optimization{opt_problem}};
  vector<double> tol(160,1e-3);
 
  prob.set_c_tol(tol);
  
  algorithm algo{snopt7(false,"/usr/local/lib/")};
  //algorithm algo{cmaes(500)};
  population pop{prob,10u};
  /*
  vector<double> x_in(160,0.0);

  for(int i=0;i<10;++i)
  {
    for (int j=0;j<16;++j)
    {
      x_in[i*16+j]=joints_0[j];
    }
  }
  vector<double> f_in=prob.fitness(x_in);
  pop.set_xf(0,x_in,f_in); 
  */
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
    /*
    if(i==-1+j_best.size()/16)
    {
      vector<double> th_j(&j_arr[12],&j_arr[12+4]);
      vector<double> th_pose=allegro_kdl.getFK(3,th_j,true);
      cerr<<"Obj pose: "<<th_pose[0]<<" "<<th_pose[1]<<" "<<th_pose[2]<<" "<<th_pose[3]<<" "<<th_pose[4]<<" "<<th_pose[5]<<endl;
    }
    */
  }

  res.joint_traj=joint_traj;
  return true;
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"in_grasp_server");
  ros::NodeHandle n;
  ros::ServiceServer service=n.advertiseService("/in_hand/primitive/in_grasp_manipulation",in_grasp_call);
  ros::spin();// Loop rate?
}
