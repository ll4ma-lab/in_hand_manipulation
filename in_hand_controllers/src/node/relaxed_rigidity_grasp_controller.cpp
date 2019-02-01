#include <hand_services/external_controller/relaxed_rigidity.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"relaxed_rigidity_grasp_controller");
  // create instance of allegro interface:
  string ns="relaxed_rigidity";
  
  robot_interface::AllegroInterface allegro_int(ns);
  allegro_int.init();

  // create control interface:
  double loop_rate=100;
  hand_task_controller::relaxedRigidityGrasp ct(ns,&allegro_int,loop_rate);
  
  
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
