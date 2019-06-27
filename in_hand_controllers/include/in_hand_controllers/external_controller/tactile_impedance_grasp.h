#include <hand_services/external_controller/in_hand_controller_template.h>
//#include <in_hand_manipulation_msgs/SurfaceNormal.h>
#include <in_hand_controllers/TactileImpedanceGrasp.h>
#include <visualization_msgs/MarkerArray.h>
#include <biotac_sensors/BioTacForce.h>
namespace hand_task_controller
{
class tactileController : inHandController
{
public:
  tactileController(string ns, robot_interface::RobotInterface *robot_interface,double loop_rate);
  
  bool configureHook();

  bool startHook(){ return 0;};
  void stopHook(){};
  void cleanupHook(){};
  void updateHook(){inHandController::updateHook();};

  
protected:
  //bool estimate_friction(const int &finger, const Eigen::Vector3d &force_pt, const Eigen::Vector3d &force_direction, const double &force_mag, Eigen::Vector3d &slip_force);
  bool get_control_cmd(Eigen::VectorXd &q_cmd ,Eigen::VectorXd &qd_cmd);
  
  ros::ServiceServer update_control_req_;
  bool update_control_srv(in_hand_controllers::TactileImpedanceGrasp::Request &req, in_hand_controllers::TactileImpedanceGrasp::Response &res);
  
  // tactile data subscrition:
  ros::Subscriber tactile_sub_,surface_sub_;
  //slip_cb(std_msgs::Int16MultiArray &msg);
private:
  Eigen::VectorXd old_ftip_pose_;
  double kin_slip_thresh_;
  double cmd_timeout_;

  bool tactileControl(Eigen::VectorXd &j_cmd);

  /** 
   * Tactile slip detection function, compares sensed force to friction cone and surface normal
   * 
   * 
   * @return true if slip is detected, false if in stable grasp
   */
  bool tactileSlip(const int &finger);

  /** 
   * Slip detection based on fingertip velocity
   * 
   * 
   * @return true if slip is detected
   */
  bool kinematicSlip(const int &finger, const Eigen::Vector3d &slip_dir);
  
  /// fingers to use for grasp control
  vector<int> f_idx;
  
  /// force messages
  vector<Eigen::VectorXd> tac_forces;

  /// Contact detection from tactile sensors
  vector<int> tac_contacts;
  vector<Eigen::Vector3d> tac_cpts;

  /** 
   * Tactile sensor message callback
   * 
   * @param msg 
   */
  void tactile_msg_callback(biotac_sensors::BioTacForce msg);

  /** 
   * Surface normal message callback
   * 
   * @param msg 
   */
  void surface_normal_callback(visualization_msgs::MarkerArray msg);
  
  
  /// Flag to indicate if a tactile message has been recieved
  bool tactile_msg_=false;
  
  vector<double> grasp_ampl_;
  Eigen::VectorXd last_jcmd_,q_servo_cmd_;
  double stiffness_delta_,deviation_force_;
  double force_radius_;

  vector<Eigen::Vector3d> surface_normal_;
  bool got_surface_normal_=false;
  //
  Eigen::VectorXd f_mag_,alpha_;
  vector<Eigen::VectorXd> des_ftip_poses_;
  /// coefficient of friction
  double mu_,mu_slip_;
};
}

