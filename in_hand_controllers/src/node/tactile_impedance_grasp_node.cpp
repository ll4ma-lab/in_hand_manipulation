#include<in_hand_controllers/external_controller/tactile_impedance_grasp.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"in_hand_controllers/tactile_impedance_grasp");
  string ns="allegro_impedance_grasp";
  robot_interface::AllegroInterface allegro_int(ns);
  allegro_int.init();

  double loop_rate=30;
  hand_task_controller::tactileController ct(ns,&allegro_int,loop_rate);

  bool control_ready=ct.configureHook();
  ROS_INFO("Control interface initialized");
  if(control_ready)
  {
    ct.updateHook();
  }
  else
  {
    cerr<<"configuration failed!"<<endl;
    return 1;
  }
  return 0;
}
