// Takes end-effector position in the palm frame as input and sends joint position command using Jacobian transpose control
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/scoped_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <ros/package.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>


namespace hand_controller {

class robotNode
{
 public:
  // Current robot state
  sensor_msgs::JointState joint_state_;
  // joint command
  sensor_msgs::JointState joint_cmd_;
  ros::Publisher joint_pub_;
  ros::Subscriber joint_sub_,biotac_sub_;
  // flags:
  bool got_robot_state,got_biotac_state;
  int init(ros::NodeHandle&,std::string);
  void stCallback(const sensor_msgs::JointState::ConstPtr&);
  void biotacCallback(const std_msgs::Float32MultiArray::ConstPtr&);
  void updateState();
  // ROS variables
  ros::NodeHandle* n_;
  void publish_joints(Eigen::VectorXd j_pos);
  std::vector<float> biotac_data;
};  

 class KDLController: public robotNode
 {
 protected:
   // Allegro chain of links and joints for all 4 fingers.
   KDL::Tree allegro_tree_;
   KDL::Chain index_chain_;
    
   // KDL solvers
   boost::scoped_ptr<KDL::ChainFkSolverPos> index_pose_solver_;
   boost::scoped_ptr<KDL::ChainJntToJacSolver> index_jac_solver_;

   // Variables to be pre-allocated
   // Postion control:
   KDL::JntArray q_; // Joint positions
   KDL::JntArray qdot_; // Joint velocities
   KDL::Jacobian  J_; // Jacobian
   KDL::Frame cart_pos_; // Cartesian position from FK
  };


}











