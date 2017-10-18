#include <ros/ros.h>
#include "in_hand_manipulation_msgs/RunMPC.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <hand_task_controllers/kdl_controller.h>
#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseArray.h>

#include <ros/package.h>

// Eigen library for vectors and matrices:
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

#include <tf_conversions/tf_kdl.h>
using namespace std;
namespace hand_controller
{
class mpcController : public robotNode
{
public:
  tf::TransformListener* tf_listen;
  manipulator_kdl::robotKDL* hand_kdl;
  vector<Eigen::VectorXd> j_des;
  vector<KDL::Frame> obj_pose_des;
  KDL::Frame curr_obj_pose;
  void init_controller()
  {
    tf::TransformListener listen;
    tf_listen=&listen;
    
    // build kdl tree
    string _urdf_file = ros::package::getPath("urlg_robots_description");
    _urdf_file.append("/robots/depreciated/urdf/allegro_kdl.urdf");
    vector<string> _ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
    vector<string> _base_names={"palm_link","palm_link","palm_link","palm_link"};
    vector<double> _g_vec={0.0,0.0,0.0};

    // build kdl tree and chains:
    hand_kdl=new manipulator_kdl::robotKDL(_urdf_file,_base_names,_ee_names,_g_vec);

    // service:
    ros::ServiceServer ss = n_->advertiseService("/in_grasp_manipulation/one_step_mpc", &mpcController::srv_call, this);



  }

  void get_object_pose()
  {
    tf::StampedTransform transform;
    try
    {
      tf_listen->lookupTransform("/palm_link", "/obj",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    tf::TransformTFToKDL(transform,curr_obj_pose);
  }
  
  // service callback:
  void set_des_traj(trajectory_msgs::JointTrajectory joint_traj, geometry_msgs::PoseArray obj_pose_arr)
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

  void publish_joint_pos(Eigen::VectorXd j_pos)
  {
    joint_cmd_.name=joint_state_.name;
    joint_cmd_.header.stamp=ros::Time::now();
    vector<double> j_arr(j_pos.data(), j_pos.data() + j_pos.size());
    joint_cmd_.position=j_arr;
    joint_pub_.publish(joint_cmd_);
  }
  void run_controller()
  {
    //
    ros::Rate loop_rate(50);
    int T=100;
    int t=1;
    while (n_->ok() && t<T)
    {
      updateState();
      get_object_pose();
      // compute feedback gain:

      // publish trajectory:
      t++;
      loop_rate.sleep();
    }
  }
  bool srv_call(in_hand_manipulation_msgs::RunMPC::Request& req,in_hand_manipulation_msgs::RunMPC::Response& res)
  {
    set_des_traj(req.joint_traj,req.obj_traj);
    // run controller:
    
    // return reached object pose
    updateState();
    get_object_pose();
    geometry_msgs::PoseStamped reached_pose;
    tf::PoseKDLToMsg(curr_obj_pose, reached_pose.pose);
    res.reached_obj_pose=reached_pose;
    sensor_msgs::JointState reached_js;
    reached_js=joint_state_;
    return true;
  }

};
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"one_step_mpc_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);
  hand_controller::mpcController robot_node;
  robot_node.init(n,"/allegro_hand_right");
  robot_node.init_controller();
  

  while(n.ok())
  {
    // get object transform:
    //robot_node.updateState();
    //robot_node.get_object_pose();
    ros::spinOnce();
    loop_rate.sleep();
  }


}



// Get desired joint trajectory, desired object pose trajectory

// Compute feedforward delta_theta

// while loop:(t<T) T=interpolated timesteps

// Compute transformation matrix between thumb and object

// Compute feedback delta_theta

// command robot

