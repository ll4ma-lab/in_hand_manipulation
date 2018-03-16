// Takes end-effector position in the palm frame as input and sends joint position command using Jacobian transpose control
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/scoped_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <ros/package.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>


namespace jacobian_controller_ns {

class PositionController
 {
 private:

    // Current robot state
    sensor_msgs::JointState* allegro_state_;

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
    double x_[3]; // End-effector position
    double xd_[3];// Desired end-effector position
    double x0_[3];// Initial end-effector position
    double x_old_[3];// Previous end-effector position
    double x_err[3];// Err in end-effector position
    double xdot_[3]; // Linear velocity of end-effector
    double Kp_[3]; // Proportional gains
    double Kd_[3]; // Derivative gains
    double gain;

    double q_control_[4];//joint position output from controller 
    //KDL::JntArray tau_;
    
    // ROS variables
    ros::Publisher joint_pub_,control_pub_;

    std_msgs::Int8 control_type_;


    sensor_msgs::JointState joint_cmd_;
    // Flags to check data flow
    bool control_init_=true;
    bool got_position_=false;
    bool initialized_controller_=false;

  public:
    int init(ros::NodeHandle&);
    void pointCallback(geometry_msgs::PointStamped);
    void stCallback(sensor_msgs::JointState);
  };


class ForceController
 {
 private:

    // Current robot state
    sensor_msgs::JointState* allegro_state_;

    // Allegro chain of links and joints for all 4 fingers.
    KDL::Tree allegro_tree_;
    KDL::Chain index_chain_;
    KDL::Chain allegro_chain_[4];
    // KDL solvers
    boost::scoped_ptr<KDL::ChainFkSolverPos> pose_solver_[4];
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_[4];
    Eigen::MatrixXd k_gains;
    Eigen::VectorXd k_jac;
    // Variables to be pre-allocated
    // Postion control:
    KDL::JntArray q_[4]; // Joint positions
    KDL::JntArray qdot_[4]; // Joint velocities
    KDL::Jacobian  J_[4]; // Jacobian
    //    KDL::Frame cart_pos_[4]; // Cartesian position from FK

    double x_[3]; // End-effector Force
    double xd_[12];// Desired end-effector Force 3x 4 fingers
    
    double x_old_[3];// Previous end-effector Force
    double x_err[3];// Err in end-effector Force

    double Kp_[3]; // Proportional gains
    double Kd_[3]; // Derivative gains
    

    double tau_control_[16];//joint torque output from controller 
    //KDL::JntArray tau_;
    
    // ROS variables
    ros::Publisher joint_pub_,control_pub_;


    sensor_msgs::JointState joint_cmd_;
    std_msgs::Int8 control_type_;
    
    // Flags to check data flow
    bool control_init_=true;
    bool got_desired_=false;
    bool initialized_controller_=false;

  public:
    int init(ros::NodeHandle&);
    void pointCallback(geometry_msgs::PoseArray);
    void stCallback(sensor_msgs::JointState);
  };
}











