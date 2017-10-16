#include <ros/ros.h>
#include "in_grasp_manipulation/RunMPC.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

// create a class:
namespace controller
{
class highLevelController
{
public:
  highLevelController(string joint_state_topic="/allegro_hand_right/joint_states", string joint_publish_topic="/allegro_hand_right/joint_cmd");
  ros::Publisher joint_pub;
  ros::Subscriber joint_cb;
  sensor_msgs::JointState curr_joint_state;  
  
}
  class mpcController: public highLevelController
  {
  public:
    void service_callback();
  }
    
}
bool mpc_controller(in_grasp_manipulation::RunMPC::Request &req,
                    in_grasp_manipulation::RunMPC::Response &res)
{
  
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"one_step_mpc_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

  ros::Publisher cmd_pub=n.advertise<sensor_msgs::JointState>("allegro_hand_right/joint_cmd",1);
  tf::TransformListener listener;
  ros::ServiceServer service = n.advertiseService("in_grasp/one_step_mpc", mpc_controller);

  while(n.ok())
  {
    // Get object transform:
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/palm_link", "/obj",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
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

