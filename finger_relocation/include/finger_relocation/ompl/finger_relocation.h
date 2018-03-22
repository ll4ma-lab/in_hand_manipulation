#include <ros/ros.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/geometric/GeneticSearch.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

class FGGoal : public ob::Goal
{
public:
  FGGoal(const ob::SpaceInformationPtr &si) : ompl::base::Goal(si);
  virtual bool isSatisfied(const ob::State *st) const;
  

  
}
