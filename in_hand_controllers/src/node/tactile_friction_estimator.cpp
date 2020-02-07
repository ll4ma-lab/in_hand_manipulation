#include<in_hand_controllers/external_controller/tactile_friction_estimation.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"in_hand_controllers_tactile_friction_estimator");
  string ns="allegro_grasp";
  robot_interface::AllegroInterface allegro_int(ns);
  allegro_int.init();

  double loop_rate=10;
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
