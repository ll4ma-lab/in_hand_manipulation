#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT true
#endif
//#include "ll4ma_planner/debug.h"
#include <iostream>


#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>
//#include <pcl_mesh_tools/collision_checker/convex_collision_checker.h>

// Eigen library for vectors and matrices:
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

using namespace std;
namespace opt_problems
{
  class optProblem
  {
  public:
    // constructor
    optProblem(int dummy);// dummy since pagmo requires default values for variables
    optProblem(int time_steps, manipulator_kdl::robotKDL &robot_kdl, vector<double> weights, vector<vector<Eigen::VectorXd>> &des_f_poses, vector<double> vel_limits,vector<double> init_joint_angles,vector<int> fingers);
    
    //optProblem(int time_steps,manipulator_kdl::robotKDL robot_kdl, vector<vector<double>> data_arr,vector<Eigen::MatrixXd> data_mats, vector<double> vel_limits,vector<double> init_joint_angles);

    //optProblem(int time_steps,manipulator_kdl::robotKDL robot_kdl,collisionChecker c_check, vector<vector<double>> data_arr,vector<double> vel_limits,vector<double> init_joint_angles);

    // functions
    double objFunction(vector<double> x) const; 
    vector<double> inEqConstraints(vector<double> x) const;
    vector<double> EqConstraints(vector<double> x) const;
    //#if SPARSE_GRADIENT==false
    vector<double> gradient(vector<double> x) const;
    //#endif
    vector<double> objGradient(vector<double> x) const;
    vector<double> inEqGradient(vector<double> x) const;
    vector<double> eqGradient(vector<double> x) const;
    
    vector<vector<double>> bounds() const;
    #if SPARSE_GRADIENT==true
    vector<double> sparse_gradient(vector<double> x) const;
    std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> gradient_sparsity() const;
    #endif
    int m_dim,m_nec,m_nic;
    vector<double> joints_0;
  private:
    manipulator_kdl::robotKDL* robot_kdl_;
    // Collision checking: TODO
    //mutable collisionChecker c_checker;
    int timesteps_;
    double wa,wd;
    double vel_lim;
    vector<vector<Eigen::VectorXd>> des_f_poses_;
    vector<int> fingers_;
    
  };
}
