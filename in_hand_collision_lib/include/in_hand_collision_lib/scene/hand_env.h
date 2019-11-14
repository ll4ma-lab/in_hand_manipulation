//#include <ll4ma_collision_wrapper/libs/fcl/collision_checker.h>
#include <future>
//#include <ll4ma_kdl/utils/transforms.h>

#include <ll4ma_opt_utils/plan_env_utils.hpp>
namespace in_hand_manipulation
{
class handEnv : public ll4ma_opt_utils::envUtils
{
public:
  handEnv():ll4ma_opt_utils::envUtils() {};
  handEnv(ros::NodeHandle &n):ll4ma_opt_utils::envUtils(n){};
  
  // functions that change the private variables of this class:
  // robot link functions:
  
  bool update_robot_state(const vector<double> &q, const bool &arm_static = true);

  bool update_arm_state(const vector<double> &q);
  // environment functions:
  
  //void update_world(PointCloud<PointXYZ>::Ptr env_cloud, PointCloud<PointXYZ>::Ptr obj_cloud);

  
  //void update_grasp_obj_pose(vector<double> &pose);
  
  
  // collision checking functions:
  //checking collision between all robot links and the environment

  void check_ftip_contact(vector<double> &pose, coll_data &c_data, int ftip_idx);

  void get_ftip_contact_pt(vector<double> &in_pose,int ftip_idx, Vector3d &c_pt);

  void update_collision_links(const vector<int> &l_idx);

  /// Check if finger links are in contact with the object:
  void check_contacts(vector<vector<double>> &l_poses, vector<vector<coll_data>> &c_data);

  /// This only returns the largest penetration or the closest collision data(when not penetrating)
  void check_contacts(vector<vector<double>> &l_poses, vector<coll_data> &c_data);

  /// Check if finger links (only l_idx) are in contact with the object, useful when only few links have to be queried
  void check_contacts(vector<vector<double>> &l_poses, vector<vector<coll_data>> &c_data,vector<int> l_idx);

  /// Compute signed distance orientation gradient by finite difference:
  //void get_orient_contacts(vector<vector<double>> &l_poses, vector<int> l_idx, vector<Eigen::VectorXd> &or_grad,double delta=0.001);
  void update_link_state(vector<double> &link_pose, int l_idx);

  void check_contacts(vector<vector<double>> &l_poses, vector<coll_data> &c_data,vector<int> l_idx);

  void check_link_collision(vector<vector<coll_data>> &c_data, vector<int> l_idx);
  void check_link_collision( int l_idx, vector<double> &l_pose, coll_data &c_data);

  //void get_link_markers(visualization_msgs::MarkerArray &link_markers,const string &frame_id);
  //void get_env_markers(visualization_msgs::MarkerArray &box_m, const string &frame_id);
  //ll4ma_kdl::transformClass transforms;
  //vector<Transform3d> hand_link_offsets_,obj_offsets_;
  int hand_dof_,arm_dof_;
  void get_robot_state(Eigen::VectorXd  &q);
  vector<double> get_robot_state() const;

private:
  //int n_hand_links_;

  //vector<CollisionObjectd*> hand_link_objs_,collision_link_objs_;
  //vector<CollisionObjectd*> env_objs_;
  //float voxel_size_;
  //vector<float> ws_min_bound_;
  //vector<float> ws_max_bound_;
  vector<double> arm_q_;
  vector<double> robot_q_;
};
}
