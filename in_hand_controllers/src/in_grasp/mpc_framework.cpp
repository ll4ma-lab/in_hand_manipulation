#include "in_hand_task_controllers/in_grasp/mpc_framework.h"
using namespace std;
namespace hand_controller
{

void mpcController::init_controller(tf::TransformListener &listen)
{
  tf_listen=&listen;
  got_obj_state=false;
  // get values from ros parameter server:
  // mpc service name:
  string _mpc_server_name;
  if (!n_->getParam("/mpc_server_name", _mpc_server_name))
  {
    ROS_ERROR("Failed to read mpc_server_name!!!");
    ros::shutdown();
  }

  // urdf file name
  string _urdf_file;
  if (!n_->getParam("/hand_urdf", _urdf_file))
  {
    ROS_ERROR("Failed to read hand_urdf!!!");
    ros::shutdown();
  }

  if (!n_->getParam("/gains/js_tracking",js_gain))
  {
    ROS_ERROR("Failed to read mpc_server_name!!!");
    ros::shutdown();
  }
  if (!n_->getParam("/gains/obj_tracking",obj_gain))
  {
    ROS_ERROR("Failed to read mpc_server_name!!!");
    ros::shutdown();
  }

  // build kdl tree
  //string _urdf_file = ros::package::getPath("urlg_robots_description");
  //_urdf_file.append("/robots/depreciated/urdf/allegro_kdl.urdf");
  vector<string> _ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> _base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<double> _g_vec={0.0,0.0,0.0};
  
  // build kdl tree and chains:
  hand_kdl=new manipulator_kdl::robotKDL(_urdf_file,_base_names,_ee_names,_g_vec);
  ss = n_->advertiseService(_mpc_server_name, &hand_controller::mpcController::srv_call, this);
  
}

void mpcController::get_object_pose()
{
  tf::StampedTransform transform;
  bool get_obj_pose=false;
  while(get_obj_pose==false && n_->ok())
  {
    try
    {
      tf_listen->lookupTransform("/palm_link", "/dbot/obj_pose",  
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    get_obj_pose=true;
  }
  tf::TransformTFToKDL(transform,curr_obj_pose);
  get_obj_pose=false;
  while(get_obj_pose==false && n_->ok())
  {
    try
    {
      tf_listen->lookupTransform("/thumb_tip", "/dbot/obj_pose",  
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    get_obj_pose=true;
  }
  tf::TransformTFToKDL(transform,curr_rigid_frame);

}
  
// service callback:
void mpcController::set_des_traj(trajectory_msgs::JointTrajectory joint_traj, geometry_msgs::PoseArray obj_pose_arr)
{
  j_des.resize(joint_traj.points.size());
  obj_pose_des.resize(joint_traj.points.size());
  
  for(int i=0;i<joint_traj.points.size();++i)
  {
    Map<Eigen::VectorXd> des_j(&joint_traj.points[i].positions[0],joint_traj.points[i].positions.size());
    j_des[i]=des_j;
    
    // get kdl frame from pose:
    KDL::Frame f;
    tf::PoseMsgToKDL(obj_pose_arr.poses[i],f);
    obj_pose_des[i]=f;
  }
}

void mpcController::publish_joint_pos(Eigen::VectorXd j_pos)
{
  joint_cmd_.name=joint_state_.name;
  joint_cmd_.header.stamp=ros::Time::now();
  vector<double> j_arr(j_pos.data(), j_pos.data() + j_pos.size());
  joint_cmd_.position=j_arr;
  joint_pub_.publish(joint_cmd_);
}

bool mpcController::srv_call(in_hand_manipulation_msgs::RunMPC::Request& req,in_hand_manipulation_msgs::RunMPC::Response& res)
{
  cerr<<"called service!"<<endl;
  set_des_traj(req.joint_traj,req.obj_traj);
  // run controller:
  run_controller();
  // return reached object pose
  updateState();
  
  //get_object_pose();
  geometry_msgs::PoseStamped reached_pose;
  tf::PoseKDLToMsg(curr_obj_pose, reached_pose.pose);
  res.reached_obj_pose=reached_pose;
  sensor_msgs::JointState reached_js;
  reached_js=joint_state_;
  return true;
}

}
