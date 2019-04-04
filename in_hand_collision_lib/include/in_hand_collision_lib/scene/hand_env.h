#include <ll4ma_collision_wrapper/libs/fcl/collision_checker.h>
#include <future>
#include <ll4ma_kdl/utils/transforms.h>
namespace fcl_ccheck
{
class handEnv : public collisionChecker
{
public:
  
  handEnv(float const &oct_resolution=0.1f);
  
  // functions that change the private variables of this class:
  // robot link functions:
  void load_hand_links(vector<string> &link_names, string &urdf_text);
  
  void update_hand_state(vector<vector<double>> &link_poses);

  // environment functions:
  //void update_world(PointCloud<PointXYZ>::Ptr env_cloud, PointCloud<PointXYZ>::Ptr obj_cloud);

  void load_obj(const string &mesh_location);
  void update_grasp_obj_pose(vector<double> &pose);
  
  
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
  void get_orient_contacts(vector<vector<double>> &l_poses, vector<int> l_idx, vector<Eigen::VectorXd> &or_grad,double delta=0.001);
  void update_link_state(vector<double> &link_pose, int l_idx);

  void check_contacts(vector<vector<double>> &l_poses, vector<coll_data> &c_data,vector<int> l_idx);

  void check_link_collision(vector<vector<coll_data>> &c_data, vector<int> l_idx);
  void check_link_collision( int l_idx, vector<double> &l_pose, coll_data &c_data);

  void get_link_markers(visualization_msgs::MarkerArray &link_markers,const string &frame_id);
  void get_env_markers(visualization_msgs::MarkerArray &box_m, const string &frame_id);
  ll4ma_kdl::transformClass transforms;
  vector<Transform3d> hand_link_offsets_,obj_offsets_;

private:
  int n_hand_links_;
  octomap::OcTree* obj_tree_;
  float  octree_resolution_;

  vector<CollisionObjectd*> hand_link_objs_,collision_link_objs_;
  vector<CollisionObjectd*> env_objs_;
  float voxel_size_;
  vector<float> ws_min_bound_;
  vector<float> ws_max_bound_;
};
}
