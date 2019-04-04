#include "hand_task_controllers/kdl_controller.h"

using namespace std;



int main(int argc, char** argv)
{
  ros::init(argc,argv,"demo_robot_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(2);
  hand_controller::robotNode robot_node;
  robot_node.init(n,"/allegro_hand_right");
  while(n.ok())
  {
    robot_node.updateState();
    if(robot_node.got_robot_state)
    {
      cerr<<"Joint state received"<<endl;
      cerr<<robot_node.joint_state_.position[0]<<endl;
    }
    loop_rate.sleep();
  }
}

