#include <in_hand_manipulation/problems/opt_problem_base.hpp>

namespace opt_problems
{
bool optProblemBase::updateTimesteps(const int &time_steps)
{
  timesteps_=time_steps;
  return true;
}
bool optProblemBase::updateRobotModel(manipulator_kdl::robotKDL &robot_kdl)
{
  robot_kdl_=&robot_kdl;
  return true;
}

bool optProblemBase::updateInitialState(const vector<double> &init_joint_angles)
{
  joints_0=init_joint_angles;
  return true;
}

}
