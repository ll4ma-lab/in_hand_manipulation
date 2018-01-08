#include "in_hand_task_controllers/finger_gait/fg.h"

void hand_controller::FGController::run_controller()
{
  ros::Rate loop_rate(60);

  Eigen::VectorXd new_cmd=initial_js;
  bool reached=false;
  // get current state:
  //cerr<<"getting current state"<<initial_js<<endl;
  for(int i=0;i<200;++i)
  {
    Map<Eigen::VectorXd> curr_js(&joint_state_.position[0],joint_state_.position.size());
    //cerr<<curr_js<<endl;
    Eigen::VectorXd err=des_js.segment(finger*4,4)-curr_js.segment(finger*4,4);
    // threshold error:
    for(int j=0;j<err.size();++j)
    {
      if(std::abs(err[j])>vel_thresh)
      {
        // float(i)/300.0
        //err[j]=std::copysign(,err[j]);
      }
    }
    //cerr<<err.transpose()<<endl;
    new_cmd.segment(finger*4,4)=curr_js.segment(finger*4,4)+err*double(i)/200.0;
    publish_joints(new_cmd);
    loop_rate.sleep();
  }
}
int main(int argc, char** argv)
{
  ros::init(argc,argv,"fg_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
  hand_controller::FGController robot_node;
  robot_node.init(n,"/allegro_hand_right");
  robot_node.init_controller();
  while(n.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
