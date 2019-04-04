#include <ll4ma_collision_wrapper/libs/fcl/collision_checker.h>
#include <future>

namespace fcl_ccheck
{
class allegroEnv : public collisionChecker
{
public:
  //allegroEnv();
  
  allegroEnv(float const &oct_resolution=0.1f);
  
  // functions that change the private variables of this class:
  // robot link functions:
  void load_hand_links(vector<string> &f_names);
  
  void update_hand_state(vector<vector<double>> &link_poses);

  // environment functions:
  //void update_world(PointCloud<PointXYZ>::Ptr env_cloud, PointCloud<PointXYZ>::Ptr obj_cloud);

  void load_obj(const string &mesh_location);
  void update_grasp_obj_pose(vector<double> &pose);
  //void get_obj_pointcloud(pcl::PointCloud<PointXYZRGB>::Ptr);
  //void get_link_pointcloud(pcl::PointCloud<PointXYZRGB>::Ptr cloud_obj1,int l_idx_);
  
  // collision checking functions:
  //checking collision between all robot links and the environment

  void check_contacts(vector<vector<double>> &l_poses, vector<vector<collision_data>> &c_data, vector<int> l_idx);
  void check_ftip_contact(vector<double> pose, vector<collision_data> &c_data, int ftip_idx);

  
  void update_collision_links(const vector<int> &l_idx);
  void check_link_collision(vector<vector<collision_data>> &c_data, vector<int> l_idx);

private:
  int n_hand_links_;
  octomap::OcTree* obj_tree_;
  float  octree_resolution_;

  vector<CollisionObjectd*> hand_link_objs_,collision_link_objs_;
  vector<CollisionObjectd*> env_objs_;
  vector<Transform3d> hand_link_offsets_;
  float voxel_size_;
  vector<float> ws_min_bound_;
  vector<float> ws_max_bound_;
};
}
