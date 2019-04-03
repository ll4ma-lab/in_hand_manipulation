#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT true
#endif
#ifndef IN_HAND_OPT_PROB
#include <ll4ma_opt_wrapper/debug.h>
#include <iostream>


#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>

// Eigen library for vectors and matrices:
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

#include <pagmo/utils/gradients_and_hessians.hpp>

using namespace std;
namespace opt_problems
{
  class optProblemBase
  {
  public:
    ///  constructor
    //optProblemBase();

    /** 
     * Update the total timesteps for the trajectory optimization.
     * 
     * @param time_steps 
     * 
     * @return 
     */
    bool updateTimesteps(const int &time_steps);

    /** 
     * Update the robot kinematics and dynamics model
     * 
     * @param robot_kdl 
     * 
     * @return 
     */
    bool updateRobotModel(manipulator_kdl::robotKDL &robot_kdl);
    

    /** 
     * Update the initial state of the robot to enable trajectory to start from the initial state.
     * 
     * @param init_joint_angles 
     * 
     * @return 
     */
    bool updateInitialState(const vector<double> &init_joint_angles);
    

    // functions
    virtual double objFunction(vector<double> x) const{}; 
    virtual vector<double> inEqConstraints(vector<double> x) const{};
    virtual vector<double> EqConstraints(vector<double> x) const{};
    virtual vector<double> gradient(vector<double> x) const{};
    virtual vector<double> objGradient(vector<double> x) const{};
    virtual vector<double> inEqGradient(vector<double> x) const{};
    virtual vector<double> eqGradient(vector<double> x) const{};
    
    virtual vector<vector<double>> bounds() const{};
    #if SPARSE_GRADIENT==true
    virtual vector<double> sparse_gradient(vector<double> x) const{};
    virtual std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> gradient_sparsity() const{};
    #endif
    int m_dim,m_nec,m_nic;
    vector<double> joints_0;
  protected:
    manipulator_kdl::robotKDL* robot_kdl_;
    int timesteps_;

    double vel_lim;

  };
}
#endif
