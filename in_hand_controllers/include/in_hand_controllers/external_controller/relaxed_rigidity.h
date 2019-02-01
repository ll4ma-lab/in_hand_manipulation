#include <hand_services/external_controller/in_hand_controller_template.h>
#include <hand_services/updateRelaxedRigidityParams.h>
namespace hand_task_controller
{
class relaxedRigidityGrasp : inHandController
{
public:
  relaxedRigidityGrasp(string ns, robot_interface::RobotInterface *robot_interface,double loop_rate):
    inHandController(ns,robot_interface,loop_rate){}
  
  bool configureHook();

  bool startHook(){ return 0;};
  void stopHook(){};
  void cleanupHook(){};

  void updateHook(){inHandController::updateHook();};
protected:
  bool get_control_cmd(Eigen::VectorXd &q,Eigen::VectorXd &qd);
  
  ros::ServiceServer update_control_req_;
  
  
private:
  bool relaxedRigidityControl(Eigen::VectorXd &j_cmd, Eigen::VectorXd &jdot_cmd);

  bool update_relaxed_rigidity_params(hand_services::updateRelaxedRigidityParams::Request &req, hand_services::updateRelaxedRigidityParams::Response &res);

  /// fingers to use for grasp
  vector<int> f_idx;
  int rigid_f;
  vector<Eigen::VectorXd> force_dirs_;
  bool rr_parameters_set_;
  Eigen::VectorXd kp_,kd_;
  double grasp_kp;
  Eigen::VectorXd init_grasp_;

};
}

