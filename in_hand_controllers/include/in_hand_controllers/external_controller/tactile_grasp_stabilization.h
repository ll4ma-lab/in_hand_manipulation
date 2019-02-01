#include <hand_services/external_controller/in_hand_controller_template.h>
#include <hand_services/TactileGraspControl.h>
namespace hand_task_controller
{
class tactileGraspStabilization : inHandController
{
public:
  tactileGraspStabilization(string ns, robot_interface::RobotInterface *robot_interface,double loop_rate):
    inHandController(ns,robot_interface,loop_rate){}
  
  bool configureHook();

  bool startHook(){ return 0;};
  void stopHook(){};
  void cleanupHook(){};

  void updateHook(){inHandController::updateHook();};

  
protected:
  bool get_control_cmd(Eigen::VectorXd &q_cmd ,Eigen::VectorXd &qd_cmd);
  
  ros::ServiceServer update_control_req_;
  bool update_params(hand_services::TactileGraspControl::Request &req, hand_services::TactileGraspControl::Response &res);
  // tactile data subscrition:
  slip_cb(std_msgs::Int16MultiArray &msg);
private:
  bool tactileControl(Eigen::VectorXd &j_cmd);

  
  /// fingers to use for grasp control
  vector<int> f_idx;
  
  
};
}

