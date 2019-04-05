#define SPARSE_GRADIENT true

#ifndef LL4MA_COLLISION_MODEL
#define LL4MA_COLLISION_MODEL true
#include <ll4ma_collision_wrapper/scene/kris_ccd/manipulation.h>
namespace coll_ns=kris_ccd_ccheck;
#endif
#include <in_grasp_manipulation/problems/in_grasp_opt_problem.hpp>

namespace opt_problems
{

vector<double> optProblem::collisionConstraint(vector<double> x) const
{
  vector<double> retval;
  return retval;  
}

vector<double> optProblem::collisionGradient(vector<double> x) const
{
  vector<double> retval;
  
  return retval;
}

}
