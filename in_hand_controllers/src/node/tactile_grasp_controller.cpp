#include <in_hand_controllers/external_controller/tactile_grasp_stabilization.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"tactile_grasp_controller");
  // create instance of allegro interface:
  string ns="allegro_grasp";
  
  robot_interface::AllegroInterface allegro_int(ns);
  allegro_int.init();

  // create control interface:
  double loop_rate=30;
  hand_task_controller::tactileGraspStabilization ct(ns,&allegro_int,loop_rate);
  
  
  // configure controller:
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
