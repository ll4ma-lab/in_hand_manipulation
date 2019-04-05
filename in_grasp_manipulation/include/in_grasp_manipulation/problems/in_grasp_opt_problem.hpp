#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT true
#endif
#ifndef LL4MA_COLLISION_MODEL
#define LL4MA_COLLISION_MODEL true
#include <ll4ma_collision_wrapper/scene/kris_ccd/manipulation.h>
namespace coll_ns=kris_ccd_ccheck;
#endif

#include <ll4ma_opt_utils/problems/opt_problem_base.hpp>

using namespace std;
namespace opt_problems
{
class optProblem: public optProblemBase
  {
  public:
    optProblem(){};
    /** 
     * Updates the data required for the optimization
     * 
     * @param vel_limits 
     * @param data_arr 
     * @param data_mats 
     * 
     * @return true if updated successfully.
     */
    bool updateData( vector<double> vel_limits, vector<vector<double>> data_arr,vector<Eigen::MatrixXd> data_mats);
    double objFunction(const vector<double> x) const;
    vector<double> inEqConstraints(vector<double> x) const;
    vector<double> EqConstraints(vector<double> x) const;
    vector<double> gradient(vector<double> x) const{};
    vector<double> objGradient(vector<double> x) const;
    vector<double> inEqGradient(vector<double> x) const;
    vector<double> eqGradient(vector<double> x) const;
    
    vector<vector<double>> bounds() const;
    #if SPARSE_GRADIENT==true
    vector<double> sparse_gradient(vector<double> x) const;
    std::vector<std::pair<vector<double>::size_type, vector<double>::size_type>> gradient_sparsity() const;
    #endif
    // debug functions:
    vector<double> debugGrad(vector<double> state) const;
    vector<double>debugFunction(const vector<double> x) const;

    // collision functions:
    #if LL4MA_COLLISION_MODEL==true
    vector<double> collisionConstraints(vector<double> x) const;
    vector<double> collisionGradient(vector<double> x) const;
    #endif
  private:
    
    // Collision checking: TODO
    //mutable collisionChecker c_checker;
    vector<Eigen::VectorXd> des_poses;
    vector<Eigen::MatrixXd> data_mats_;
    vector<vector<double>> data_arr_;// data array for optimization
    vector<double> rrGradient(vector<double> x) const;

  };
}
