#include <pcl_mesh_tools/libs/fcl/collision_checker.h>
#include <future>

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

  void check_ftip_contact(vector<double> &pose, vector<coll_data> &c_data, int ftip_idx);

  
  void update_collision_links(const vector<int> &l_idx);

  /// Check if finger links are in contact with the object:
  void check_contacts(vector<vector<double>> &l_poses, vector<vector<coll_data>> &c_data);

  /// Check if finger links (only l_idx) are in contact with the object, useful when only few links have to be queried
  void check_contacts(vector<vector<double>> &l_poses, vector<vector<coll_data>> &c_data,vector<int> l_idx);
  
  void check_link_collision(vector<vector<coll_data>> &c_data, vector<int> l_idx);

  void get_link_markers(visualization_msgs::MarkerArray &link_markers,const string &frame_id);
  void get_env_markers(visualization_msgs::MarkerArray &box_m, const string &frame_id);

private:
  int n_hand_links_;
  octomap::OcTree* obj_tree_;
  float  octree_resolution_;

  vector<CollisionObjectd*> hand_link_objs_,collision_link_objs_;
  vector<CollisionObjectd*> env_objs_;
  vector<Transform3d> hand_link_offsets_,obj_offsets_;
  float voxel_size_;
  vector<float> ws_min_bound_;
  vector<float> ws_max_bound_;
};
}
