#include <ll4ma_collision_wrapper/libs/collision_checker/convex_collision_checker.h>
#include <future>

class allegroEnv : public collisionChecker
{
public:
  allegroEnv();
  allegroEnv(int n_links, float downsample_size, float narrow_vsize, float broad_vsize,  vector<float> min_bound={-1.2,-1.2,0.1},  vector<float> max_bound={1.2,1.2,1.2});
  // functions that change the private variables of this class:
  // robot link functions:
  void load_robot_links(vector<string> &f_names);
  void load_hand_links(vector<string> &f_names);
  
  void update_robot_state(vector<vector<double>> &link_poses);
  void update_hand_state(vector<vector<double>> &link_poses);

  // environment functions:
  void update_world(PointCloud<PointXYZ>::Ptr env_cloud, PointCloud<PointXYZ>::Ptr obj_cloud);
  // collision checking functions:
  //checking collision between all robot links and the environment
  void rob_env_ccheck(vector<vector<double>> &l_poses, vector<vector<collision_data>> &c_data);

  void grasp_contact_check(vector<vector<double>> &l_poses, vector<vector<collision_data>> &c_data, vector<int> l_idx);

  vector<collision_data> contact_fn(const obj_t &link_obj, const vector<obj_t> &narrow_env_objs, const ccd_t &ccd) const;

  
  // threading helpers:
  void single_threaded(const vector<obj_t> &links,const vector<obj_t> &broad_env_objs, const vector<obj_t> &narrow_env_objs,const ccd_t&, vector<vector<collision_data>> &c_data) const;
  void multi_threaded(const vector<obj_t> &links,const vector<obj_t> &broad_env_objs, const vector<obj_t> &narrow_env_objs,const ccd_t&, vector<vector<collision_data>> &c_data) const;
  vector<collision_data> multi_thrd_fn(const obj_t &link_obj, const vector<obj_t> &broad_env_objs, const vector<obj_t> &narrow_env_obj, const ccd_t &ccd) const;
  void multi_threaded_contact(const vector<obj_t> &link_objs, const vector<obj_t> &env_objs, vector<int> l_idx, const ccd_t &ccd,vector<vector<collision_data>> &c_data) const;
  void single_contact(const vector<obj_t> &link_objs, const vector<obj_t> &env_objs, vector<int> l_idx, const ccd_t &ccd,vector<vector<collision_data>> &c_data) const;

  ccd_t ccd;
  
private:
  int n_links_;
  int n_hand_links_;
  
  vector<vector<Vector3d>> link_meshes_;
  vector<vector<Vector3d>> hand_link_meshes_;
  vector<vector<Vector3d>> env_meshes_;
  vector<vector<Vector3d>> obj_meshes_;
  vector<obj_t> obj_;
  vector<obj_t> link_objs_;
  vector<obj_t> hand_link_objs_;
  vector<obj_t> narrow_env_objs_;
  vector<obj_t> broad_env_objs_;

  float broad_vsize_;
  float narrow_vsize_;
  float voxel_size_;
  vector<float> ws_min_bound_;
  vector<float> ws_max_bound_;
};
