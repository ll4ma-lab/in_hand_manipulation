#include "hand_task_controllers/kdl_controller.h"


namespace hand_controller
{
  void robotNode::stCallback(const sensor_msgs::JointState::ConstPtr& js)
{
  joint_state_=*js;
  got_robot_state=true;
}
void robotNode::updateState()
{
  ros::spinOnce();
}
void robotNode::biotacCallback(const std_msgs::Float32MultiArray::ConstPtr& b_dat)
{
  biotac_data.resize(b_dat->data.size());
  for(int i=0;i<b_dat->data.size();++i)
  {
    biotac_data[i]=b_dat->data[i];
  }
  got_biotac_state=true;
}
int robotNode::init(ros::NodeHandle& n, std::string robot_prefix)
{
  std::string joint_cmd_topic="/joint_cmd";
  std::string joint_state_topic="/joint_states";
  // create ros publisher
  joint_pub_=n.advertise<sensor_msgs::JointState>(joint_cmd_topic.insert(0,robot_prefix),1);
  // create ros subscriber
  joint_sub_=n.subscribe<sensor_msgs::JointState>(joint_state_topic.insert(0,robot_prefix), 1,&robotNode::stCallback,this);
  biotac_sub_=n.subscribe<std_msgs::Float32MultiArray>("biotac/sigmoid",1,&robotNode::biotacCallback,this);
  got_robot_state=false;
  got_biotac_state=false;
  std::string names[]={"index_joint_0","index_joint_1","index_joint_2", "index_joint_3",
                   "middle_joint_0","middle_joint_1","middle_joint_2", "middle_joint_3",
                   "ring_joint_0","ring_joint_1","ring_joint_2", "ring_joint_3",
                   "thumb_joint_0","thumb_joint_1","thumb_joint_2", "thumb_joint_3"};
 
  std::vector<std::string> name_vect(names,names+sizeof(names)/sizeof(std::string)); //make it const if you can
  joint_cmd_.name=name_vect;
  joint_cmd_.position.resize(16);
  
  // create loop rate
  n_=&n;
}
void robotNode::publish_joints(Eigen::VectorXd j_pos)
{
  std::vector<double> q(j_pos.data(),j_pos.data()+j_pos.size());
  joint_cmd_.position=q;
  joint_pub_.publish(joint_cmd_);
}

}
