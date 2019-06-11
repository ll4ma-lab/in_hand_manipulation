#include<in_hand_controllers/external_controller/tactile_friction_estimation.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"in_hand_controllers/tactile_friction_estimator");
  string ns="allegro_grasp";
  robot_interface::AllegroInterface allegro_int(ns);

  double loop_rate=30;
  hand_task_controller::tactileController ct(ns,&allegro_int,loop_rate);

  bool control_ready=ct.configureHook();
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
