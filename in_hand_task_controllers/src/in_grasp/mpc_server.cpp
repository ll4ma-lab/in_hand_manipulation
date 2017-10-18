#include "in_hand_task_controllers/in_grasp/mpc_framework.h"

int main(int argc, char** argv)
{
  ros::init(argc,argv,"one_step_mpc_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
  hand_controller::mpcController robot_node;
  robot_node.init(n,"/allegro_hand_right");
  robot_node.init_controller();


  while(n.ok())
  {
    // controller is called inside the service, this loop just looks for a new service request
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

