//#include "ll4ma_planner/debug.h"
#include <iostream>

//#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>

#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT true
#endif

#ifndef LL4MA_COLLISION_MODEL
#define LL4MA_COLLISION_MODEL true
#include <ll4ma_collision_wrapper/scene/kris_ccd/manipulation.h>
namespace coll_ns=kris_ccd_ccheck;
#endif
#include <ll4ma_opt_utils/problems/opt_problem_base.hpp>
#include <in_hand_collision_lib/scene/hand_env.h>

using namespace std;
//using namespace fcl_cchec;
namespace opt_problems
{
class optProblem: public optProblemBase
{ 
public:
  // constructor
  optProblem();// dummy since pagmo requires default values for variables

  bool updateData(const Eigen::VectorXd &des_pose, const vector<double> &vel_limits, const int &finger);

  bool updateScene(in_hand_manipulation::handEnv &hand_env);
  
  // functions
  double objFunction(vector<double> x) const; 
  vector<double> inEqConstraints(vector<double> x) const;
  vector<double> EqConstraints(vector<double> x) const;
  vector<double> gradient(vector<double> x) const;
  vector<double> objGradient(vector<double> x) const;
  vector<double> inEqGradient(vector<double> x) const;
  vector<double> eqGradient(vector<double> x) const;
  vector<double> collisionGradient(vector<double> x) const;
  vector<double> collisionGradientFD(vector<double> x) const;

  //Eigen::MatrixXd compute_jacobian(int l_idx, Eigen::VectorXd q_state, Eigen::VectorXd c_pt) const;

  vector<vector<double>> bounds() const;
#if SPARSE_GRADIENT==true
  vector<double> sparse_gradient(vector<double> x) const;
  std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> gradient_sparsity() const;
#endif
  int f_tip_;
  vector<int> l_idx_;
  double dist_thresh_,stability_thresh_;
private:
  //int timesteps_;
  Eigen::VectorXd des_pose_,initial_pose_;
  Eigen::VectorXd init_joints;
  //double vel_lim;
  int finger_;
  mutable in_hand_manipulation::handEnv *hand_env_;
  int hand_dof_;

  };
}
