#include<finger_relocation/problem/rrt_finger_gait.h>


bool fgGoal::setup_planner(manipulator_kdl::robotKDL robot_kdl,handEnv c_check)
{
  robot_kdl_=robot_kdl;
  dim=robot_kdl_.dof;
  c_check_=c_check;
  return true;
}

bool fgGoal::setup_goal(Eigen::VectorXd g_pt, int f_idx)
{
  goal_pt=g_pt;
  f_idx_=f_idx;
}

bool fgGoal::isSatisfied(const ob::State *st) const
{
  // get ik
  Eigen::VectorXd q_state(dim);
  for(int i=0;i<dim;++i)
  {
    q_state[i]=st->as<ob::RealVectorStateSpace::StateType>()->values[i];
  }
  Eigen::VectorXd ee_pose;
  robot_kdl_.getFK(f_idx_,q_state,ee_pose);

  double d=(goal_pt-ee_pose.head(3)).squaredNorm();
  if(d<1e-5)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// this function computes the distance to the desired pose:
bool fgGoal::isSatisfied(const ob::State *st, double *distance) const
{
  Eigen::VectorXd q_state(dim);
  for(int i=0;i<dim;++i)
  {
    q_state[i]=st->as<ob::RealVectorStateSpace::StateType>()->values[i];
  }
  Eigen::VectorXd ee_pose;
  robot_kdl_.getFK(f_idx_,q_state,ee_pose);

  double d=(goal_pt-ee_pose.head(3)).squaredNorm();
  *distance=d;
  if(d<1e-5)
  {
    return true;
  }
  else
  {
    return false;
  }
}



// check for validity, bounds are already set, so only check for collisions?
bool fgStateChecker::isValid(const ob::State *st) const
{
  // update link pose and check for collisions
  vector<vector<double>> link_poses;
  link_poses.resize(3*4);

  vector<int> l_idx_={f_idx_*3,f_idx_*3+1};
  // get link pose for every link:
  Eigen::VectorXd finger_q;
  finger_q.resize(4);
  
  for(int i=0;i<dim;++i)
  {
    finger_q[i]=st->as<ob::RealVectorStateSpace::StateType>()->values[i];
  }

  Eigen::VectorXd l_pose_;
  
  for (int j=0;j<4;j++)
  {
    //finger joints:
    if(j==f_idx_)
    {
      for(int i=0;i<3;++i)
      {
      robot_kdl_.getLinkPose(j*3+i,finger_q,l_pose_);
      link_poses[j*3+i].resize(l_pose_.size());
      Eigen::VectorXd::Map(&link_poses[j*3+i][0], l_pose_.size()) = l_pose_;

      }
    }
    else
    {
      for(int i=0;i<3;++i)
      {
        link_poses[j*3+i].resize(7,0.0);
      }
    }
  }
    
  // get contact data:
  vector<coll_data> contact_data;
  c_check_.check_contacts(link_poses,contact_data,l_idx_);
  for(int i=0;i<l_idx_.size();++i)
  {
    if(contact_data[l_idx_[i]].collision)
    {
      return false;
    }
  }
  return true;
}
namespace ompl_wrapper
{
void fgProblem::setupGoal(Eigen::VectorXd goal_pt,int f_idx, handEnv c_check)
{

  // setup goal and state validity classes:
  fgGoal* goal(new fgGoal(ss->getSpaceInformation()));
  //const fgStateChecker* fg_checker(new );
  ss->setStateValidityChecker(make_shared<fgStateChecker>(ss->getSpaceInformation(),robot_kdl_,c_check,f_idx));
  
  goal->setup_planner(robot_kdl_,c_check);

  goal->setup_goal(goal_pt,f_idx);

  ob::GoalPtr goal_ptr(goal);
  ss->setGoal(goal_ptr);

}
}
