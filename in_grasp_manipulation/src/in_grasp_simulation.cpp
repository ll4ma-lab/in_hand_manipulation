#include "in_grasp_manipulation/in_grasp_sim.h"

#include "in_grasp_manipulation/InGraspSim.h"


int main(int argc, char** argv)
{
  ros::init(argc,argv,"in_grasp_sim");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  sensor_msgs::JointState curr_state;
  std::string names[]={"index_joint_0","index_joint_1","index_joint_2", "index_joint_3",
                   "middle_joint_0","middle_joint_1","middle_joint_2", "middle_joint_3",
                   "ring_joint_0","ring_joint_1","ring_joint_2", "ring_joint_3",
                   "thumb_joint_0","thumb_joint_1","thumb_joint_2", "thumb_joint_3"};
  
  
  std::vector<std::string> name_vect(names,names+sizeof(names)/sizeof(std::string)); //make it const if you can
  curr_state.name=name_vect;
  in_grasp_sim::inGraspSim robot_sim;
  robot_sim.init(n);
  robot_sim.init_service(n);
  
  while(n.ok())
  {
    if(robot_sim.setup_obj_frame)
    {
      robot_sim.update_in_grasp_state();
    }
    robot_sim.publish_state();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
            
}
