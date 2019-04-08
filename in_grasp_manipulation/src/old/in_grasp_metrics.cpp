#define SPARSE_GRADIENT true
// optimization:
#include <ll4ma_planner/pagmo_opt/trajectory_optimization.hpp>

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
  // TODO: ros::package segfaults, fix!
  string urdf_file="/home/bala/catkin_ws/src/in_hand_manipulation/in_grasp_manipulation/urdf/allegro.urdf";

  // Load allegro kdl object:
  vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics

  manipulator_kdl::robotKDL allegro_kdl(urdf_file,base_names,ee_names,g_vec);
  allegro_kdl.urdfParser();
  // Getting parameters from service request:

  vector<double> joints_0;
  vector<double> final_joints;
  vector<Eigen::MatrixXd> data_mat;
  joints_0.resize(16);
  final_joints.resize(16);
  for(int i=0;i<req.init_joints.position.size();++i)
  {
    joints_0[i]=req.init_joints.position[i];
  }
  
  if(req.final_joints.position.size()!=0)
  {
    for(int i=0;i<req.init_joints.position.size();++i)
    {
      final_joints[i]=req.final_joints.position[i];
    }
  }
  else
  {
    final_joints=joints_0;
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
    obj_mat(i,1)=R[i][1];
    obj_mat(i,2)=R[i][2];
  }
  obj_mat(0,3)=req.object_pose.position.x;
  obj_mat(1,3)=req.object_pose.position.y;
  obj_mat(2,3)=req.object_pose.position.z;

  
  vector<double> thumb_joints;
  thumb_joints.push_back(joints_0[12]);
  thumb_joints.push_back(joints_0[13]);
  thumb_joints.push_back(joints_0[14]);
  thumb_joints.push_back(joints_0[15]);
  Eigen::MatrixXd thumb_mat=allegro_kdl.getFKEigen(3,thumb_joints);
  
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
  
  vector<double> r_fin;
  int r_f=req.rigid_finger;
  r_fin.resize(1,r_f);
  data_arr.resize(4+fingers.size()+1); 
  data_arr[0]=des_pose;
  data_arr[2]=r_fin;
  data_arr[3]=fingers_d;

  vector<double> thumb_pose_arr=allegro_kdl.getFK(r_f,thumb_joints,true);
  data_arr[4+fingers.size()]=thumb_pose_arr;

  Map<Eigen::VectorXd> thumb_pose(&thumb_pose_arr[0],thumb_pose_arr.size());;
  // compute offsets in thumb frame:
  // thumb matrix w_T_th=thumb_mat
  //compute inverse:
  Eigen::MatrixXd thumb_inv=thumb_mat.inverse();
  for(int i=0;i<fingers.size();++i)
  {
    int f=int(fingers_d[i]);
    // get joint angles for finger f:
    vector<double> f_joints(&joints_0[f*4],&joints_0[f*4+4]);

    //get finger mat w_T_f
    Eigen::MatrixXd f_mat=allegro_kdl.getFKEigen(f,f_joints);

    // Compute offset matrix:
    Eigen::MatrixXd offset_mat=thumb_inv*f_mat;
    data_mat.push_back(offset_mat);
    vector<double> f_pose_arr=allegro_kdl.getFK(f,f_joints,true);
    Map<Eigen::VectorXd> f_pose(&f_pose_arr[0],f_pose_arr.size());
    Eigen::VectorXd pose_diff=thumb_pose-f_pose;
    vector<double> position_offset;
    position_offset.resize(pose_diff.size());
    
    pose_diff[0]=offset_mat(0,3);
    pose_diff[1]=offset_mat(1,3);
    pose_diff[2]=offset_mat(2,3);
    
    // Compute orientation offset:

    vector<double> thumb_orient(thumb_pose_arr.data()+3, thumb_pose_arr.data()+6);
    vector<double> f_orient(f_pose_arr.data()+3, f_pose_arr.data()+6);
    
    vector<double> orient_offset=allegro_kdl.euler_diff(f_orient,thumb_orient);
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


  vector<double> x_in(16*t_steps_,0.0);
  
  // seed using previous joint trajectory:
  if(req.initial_jtraj.points.size()==t_steps_+1)
  {
    for(int i=0;i<t_steps_;++i)
    {
      for (int j=0;j<16;++j)
      {
        x_in[i*16+j]=req.initial_jtraj.points[i+1].positions[j];
      }
    }
  }

  
  vector<double> weights(begin(req.weights),end(req.weights));
  weights[2]=0.0;
  weights[3]=0.0;
  data_arr[1]=weights;
  // Creating opt problem instance:
  opt_problems::optProblem opt_problem(t_steps_,allegro_kdl,data_arr,data_mat,vel_limits,joints_0);
  problem prob{trajectory_optimization{opt_problem}};
  vector<double> tol(16*(t_steps_),1e-3);
  prob.set_c_tol(tol);
  algorithm algo{snopt7(false,"/usr/local/lib/")};
  population pop{prob,1u};  
  
  // compute metrics:
  // RR cost:
  vector<double> f_in=prob.fitness(x_in);
  res.metrics.resize(1);
  res.metrics[0]=f_in[0];
  return true;
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"in_grasp_metrics_server");
  ros::NodeHandle n;
  ros::ServiceServer service=n.advertiseService("/in_hand/primitive/in_grasp_manipulation/metrics",in_grasp_call);
  
  ros::spin();// Loop rate?
}
