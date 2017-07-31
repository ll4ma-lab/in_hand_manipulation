//

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

  // Getting parameters from service request:

  vector<double> joints_0;
  joints_0.resize(16);
  for(int i=0;i<req.init_joints.position.size();++i)
  {
    joints_0[i]=req.init_joints.position[i];
  }

  
  vector<vector<double>> data_arr;
  // data_arr =[des_pose;obj_frame]

  // Get initial object to thumb frame:
  tf::Pose init_pose;
  tf::poseMsgToTF(req.object_pose,init_pose);
  // init_pose->4x4
  tf::Matrix3x3 R;
  R=init_pose.getBasis();
  
  Eigen::MatrixXd obj_mat(4,4);
  obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    obj_mat(i,0)=R[i].x();
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
  vector<double> des_pose=allegro_kdl.frame_to_pose_(des_thumb_mat,true);
  // Transform quaternion to euler:
  
  data_arr.resize(2);
  vector<double> obj_frame;
  obj_frame.resize(6);
  // Create obj frame from thumb:

  data_arr[0]=des_pose;
  data_arr[1]=obj_frame;

  vector<double> vel_limits;
  vel_limits.resize(16,0.2);
  // Creating opt problem instance:
  
  opt_problems::optProblem opt_problem(10,allegro_kdl,data_arr,vel_limits,joints_0);
  // 
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"in_grasp_server");
  ros::NodeHandle n;
  ros::ServiceServer service=n.advertiseService("/in_hand/primitive/in_grasp_manipulation",in_grasp_call);
  ros::spin();// Loop rate?
}
