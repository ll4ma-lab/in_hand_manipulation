#include "in_hand_task_controllers/in_grasp/mpc_framework.h"
#include <fstream>


// This is the main control loop, declared in the mpc_framework.h file and defined here for flexibility
void hand_controller::mpcController::run_controller()
{
  //
  ros::Rate loop_rate(60);
  int T=j_des.size()-1;
  int t=0;
  // get initial rigid frame:

  js_gain=0.1;
  obj_gain=0.0;
  float obj_err_data,final_obj_err_data;
  obj_err_data=0.0;
  while (n_->ok() && t<T)
  {
    updateState();
    
    // Compute new command
    // Joint trajectory tracking:
    Map<Eigen::VectorXd> curr_js(&joint_state_.position[0],joint_state_.position.size());
    
    Eigen::VectorXd js_ff=j_des[t+1]-j_des[t];
    
    Eigen::VectorXd js_fb=1.0*(j_des[t]-curr_js);
    
    //cerr<<"JOINT ERROR: "<<js_fb.lpNorm<1>()<<endl;
    js_fb*=js_gain;

    // Object pose tracking:

    
    // publish trajectory:
    Eigen::VectorXd js_cmd=j_des[t]+js_ff+js_fb;
    publish_joints(js_cmd);
    t++;
    loop_rate.sleep();
   
  }
  cerr<<"Controller done!"<<endl;
  vector<double> err_arr;
  err_arr.emplace_back(obj_err_data);
  err_arr.emplace_back(final_obj_err_data);
  //err_=err_arr;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"one_step_mpc_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
  hand_controller::mpcController robot_node;
  robot_node.init(n,"/allegro_hand_right");
  tf::TransformListener listen;
  robot_node.init_controller(listen);
  

  while(n.ok())
  {
    // controller is called inside the service, this loop just looks for a new service request
    ros::spinOnce();
    loop_rate.sleep();
  }


}

