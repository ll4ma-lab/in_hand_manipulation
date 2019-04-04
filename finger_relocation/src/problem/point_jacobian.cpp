#include "finger_relocation/problem/finger_gait.h"
using namespace opt_problems;

Eigen::MatrixXd optProblem::compute_jacobian(int l_idx, Eigen::VectorXd q_state, Eigen::VectorXd c_pt) const
{
  Eigen::MatrixXd grad;
  grad.resize(q_state.size(),3);

  Eigen::MatrixXd cpt_mat(4,4);
  cpt_mat.setIdentity();
  cpt_mat(0,3)=c_pt[0];
  cpt_mat(1,3)=c_pt[1];
  cpt_mat(2,3)=c_pt[2];
  
  
  Eigen::MatrixXd l_mat;
  robot_kdl_->getLinkPose(l_idx,q_state,l_mat);

  // build t mat
  Eigen::MatrixXd l_T_p=l_mat.inverse()*cpt_mat;

  double eps=0.1;
  Eigen::VectorXd p_delta(q_state.size());
  //Eigen::VectorXd n_delta(q_state.size());

  // Compute gradient using fd:
  for(int i=0;i<q_state.size();++i)
  {
    p_delta.setZero();
    p_delta[i]=eps;
    // +ve change:    
    robot_kdl_->getLinkPose(l_idx,q_state+p_delta,l_mat);
    Eigen::VectorXd p_pose=(l_mat*l_T_p).col(3).head(3);
    // Transform link pose to point pose:

    // -ve change:
    robot_kdl_->getLinkPose(l_idx,q_state-p_delta,l_mat);
    Eigen::VectorXd n_pose=(l_mat*l_T_p).col(3).head(3);
    grad.row(i)=(p_pose-n_pose)/(2*eps);
  }
  return grad;
}
