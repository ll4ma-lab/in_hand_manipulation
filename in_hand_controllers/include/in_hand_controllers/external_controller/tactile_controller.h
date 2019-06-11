#include <hand_services/external_controller/in_hand_controller_template.h>
#include <in_hand_controllers/TactileGraspControl.h>
#include <biotac_sensors/BioTacForce.h>
#include <grasp_utils/eigen_grasp.hpp>
namespace hand_task_controller
{
class tactileController : inHandController
{
public:
  tactileGraspStabilization(string ns, robot_interface::RobotInterface *robot_interface,double loop_rate);
  
  bool configureHook();

  bool startHook(){ return 0;};
  void stopHook(){};
  void cleanupHook(){};

  void updateHook(){inHandController::updateHook();};

  
protected:
  bool get_control_cmd(Eigen::VectorXd &q_cmd ,Eigen::VectorXd &qd_cmd);
  
  ros::ServiceServer update_control_req_;
  bool update_params(in_hand_controllers::TactileGraspControl::Request &req, in_hand_controllers::TactileGraspControl::Response &res);
  
  // tactile data subscrition:
  ros::Subscriber tactile_sub_;
  //slip_cb(std_msgs::Int16MultiArray &msg);
private:
  bool tactileControl(Eigen::VectorXd &j_cmd);

  /** 
   * Tactile slip detection functino
   * 
   * 
   * @return true if slip is detected, false if in stable grasp
   */
  bool tactileSlip();
  
  /// fingers to use for grasp control
  vector<int> f_idx;
  
  /// force messages
  vector<Eigen::VectorXd> tac_forces;

  /// Contact detection from tactile sensors
  vector<int> tac_contacts;

  /** 
   * Tactile sensor message callback
   * 
   * @param msg 
   */
  void tactile_msg_callback(biotac_sensors::BioTacForce msg);

  /// Flag to indicate if a tactile message has been recieved
  bool tactile_msg_=false;
  
  grasping::eigenGrasp eigen_grasp_;

  vector<double> grasp_ampl_;
  Eigen::VectorXd last_jcmd_,q_servo_cmd_;
  double stiffness_delta_,deviation_force_;
};
}

