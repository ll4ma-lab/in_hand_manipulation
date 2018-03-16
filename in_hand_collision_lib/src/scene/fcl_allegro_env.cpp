#include <in_hand_collision_lib/scene/fcl_allegro_env.h>

namespace fcl_ccheck
{
// constructor:
handEnv::handEnv(const float &oct_resolution)
{
  octree_resolution_=oct_resolution;
}

void handEnv::load_hand_links(vector<string> &link_names, string &urdf_text)
{
  n_hand_links_=link_names.size();
  readLinkShapes(urdf_text, link_names, hand_link_objs_, hand_link_offsets_);

}

void handEnv::update_hand_state(vector<vector<double>> &link_poses)
{
  vector<Transform3d> l_poses;
  add_offsets(hand_link_offsets_,link_poses,l_poses);
  update_pose(hand_link_objs_,l_poses);
}

void handEnv::load_obj(const string &mesh_file_location)
{
  PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new PointCloud<pcl::PointXYZ>);
  _pcl_eigen.loadPLY(mesh_file_location,obj_cloud);

  _pcl_eigen.loadPCLOctree(obj_cloud,obj_tree_);
  // create collision boxes from octree:
  env_objs_=loadOctreeCollObj(obj_tree_);
  cout<<"Object voxel boxes size: "<<env_objs_.size()<<endl;
  
}

void handEnv::update_grasp_obj_pose(vector<double> &pose)
{
  update_pose(env_objs_,pose);
}



void handEnv::check_ftip_contact(vector<double> &pose, vector<coll_data> &c_data, int ftip_idx)
{
  Transform3d f_pose;
  add_offsets(hand_link_offsets_[ftip_idx],pose,f_pose);
  update_pose(hand_link_objs_[ftip_idx],f_pose);
  
  // check collision data
  signedDistance(hand_link_objs_[ftip_idx],env_objs_,c_data);

  
}


void handEnv::update_collision_links(const vector<int> &l_idx)
{
  // we load links against which collisions have to be checked in a seperate vector for easy collision checking:
  // thanks to pointers, we only store references, and when the link poses are updated, this list will also be updated. 
  collision_link_objs_.resize(l_idx.size());
  for(int i=0;i<l_idx.size();++i)
  {
    collision_link_objs_[i]=hand_link_objs_[l_idx[i]];
  }
  
}

void handEnv::check_link_collision(vector<vector<coll_data>> &c_data, vector<int> l_idx)
{
  //update_hand_state(l_poses);

  c_data.resize(n_hand_links_);
  for(int i=0;i<collision_link_objs_.size();++i)
  {
    signedDistance(collision_link_objs_[i],hand_link_objs_,c_data[i]);
  }
}


void handEnv::check_contacts(vector<vector<double>> &l_poses, vector<vector<coll_data>> &c_data,vector<int> l_idx)
{
  //cerr<<"updating hand state"<<endl;
  // update robot link poses:
  
  update_hand_state(l_poses);
  
  c_data.resize(n_hand_links_);
  
  // get contact data
  for(int i=0;i<l_idx.size();++i)
  {
    signedDistance(hand_link_objs_[l_idx[i]], env_objs_,c_data[l_idx[i]]);
  }
}

}
