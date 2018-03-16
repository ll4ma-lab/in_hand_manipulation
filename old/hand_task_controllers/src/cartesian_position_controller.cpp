#include "allegro_op_space_control/jacobian_controller.h"
using namespace jacobian_controller_ns;
int main(int argc , char** argv)
{

  ros::init(argc,argv,"cartesian_position_node");
  ros::NodeHandle n;
  jacobian_controller_ns::PositionController pos_control;
  pos_control.init(n);
  ros::Subscriber cmd_sub_=n.subscribe <geometry_msgs::PointStamped>("/allegro_hand_right/point_cmd",1000,&PositionController::pointCallback, &pos_control);
  ros::Subscriber joint_sub=n.subscribe<sensor_msgs::JointState>("/allegro_hand_right/joint_states",1000,&PositionController::stCallback,&pos_control);
  ;
  ros::Rate loop_rate(100);// Hz
  while(n.ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
      

  return 0;

}
