#include <ll4ma_opt_utils/problems/opt_problem_base.hpp>

using namespace std;

namespace opt_problems
{
class optProblem: public optProblemBase
{
public:
  // constructor
  optProblem();// dummy since pagmo requires default values for variables

  bool updateData(Eigen::VectorXd des_palm_pose, vector<double> vel_limits, vector<int> contact_links, vector<double> preshape,int reach_idx);
  
  //optProblem(vector<int> time_steps, manipulator_kdl::robotKDL &arm_hand_kdl, manipulationEnv &c_check, vector<double> init_joint_angles,);

    
    
  // functions
  double objFunction(vector<double> x) const; 
  vector<double> inEqConstraints(vector<double> x) const;
  vector<double> EqConstraints(vector<double> x) const;
  
  vector<double> gradient(vector<double> x) const;
  vector<double> objGradient(vector<double> x) const;
  vector<double> inEqGradient(vector<double> x) const;
  vector<double> eqGradient(vector<double> x) const;

  vector<vector<double>> bounds() const;
  
  int t_1;
    
private:
  vector<int> f_idx_;
  Eigen::VectorXd des_palm_pose_;

  vector<Eigen::VectorXd> des_poses;
  vector<Eigen::MatrixXd> data_mats_;
  vector<vector<double>> data_arr_;// data array for optimization

  double wa,wd,wp,wp2,wp_world,wm;
  vector<double> w_orient;
  Eigen::VectorXd hand_preshape;
  double coll_thresh;
  vector<double> arm_joints;
  int reach_idx_;
  double w_coll;
  double c_tol;
  int arm_dof;
};
}
