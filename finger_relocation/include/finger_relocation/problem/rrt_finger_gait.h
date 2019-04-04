#include <ompl_wrapper/rrt_base.h>
#include <in_hand_collision_lib/scene/hand_env.h>

using namespace fcl_ccheck;
namespace ompl_wrapper
{
class fgProblem : public IKompl
{
public:
void setupGoal(Eigen::VectorXd goal_pt,int f_idx,handEnv c_check);
};
}


class fgGoal : public ompl::base::Goal
{
public:
fgGoal(const ob::SpaceInformationPtr &si) : ompl::base::Goal(si)
{
}

bool setup_planner(manipulator_kdl::robotKDL robot_kdl, handEnv c_check);
bool setup_goal(Eigen::VectorXd g_pt, int f_idx);
manipulator_kdl::robotKDL robot_kdl_;
handEnv c_check_;
Eigen::VectorXd goal_pt;
int dim=0;
double thresh=0.1;
int f_idx_;
virtual bool isSatisfied(const ob::State *st) const;

// this function computes the distance to the desired pose:
virtual bool isSatisfied(const ob::State *st, double *distance) const;
};


// state validity checker class:
class fgStateChecker : public ob::StateValidityChecker
{
public:
double thresh=0.01;
mutable handEnv c_check_;
int dim=0;
manipulator_kdl::robotKDL robot_kdl_;
//bool setup_planner(manipulator_kdl::robotKDL robot_kdl, handEnv &c_check,int f_idx);
int f_idx_;
fgStateChecker(const ob::SpaceInformationPtr &si, manipulator_kdl::robotKDL r_kdl, handEnv c_check,int f_idx) : ob::StateValidityChecker(si){
robot_kdl_=r_kdl;
c_check_=c_check;
dim=r_kdl.dof;
f_idx_=f_idx;
}
  // check for validity, bounds are already set, so only check for collisions?
virtual bool isValid(const ob::State *state) const;
};

