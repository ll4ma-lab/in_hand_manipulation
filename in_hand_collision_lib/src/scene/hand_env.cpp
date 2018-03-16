#include <in_hand_collision_lib/scene/hand_env.h>

namespace fcl_ccheck
{
// constructor:
handEnv::handEnv(const float &oct_resolution)
{
  octree_resolution_=oct_resolution;
  octomap::OcTree* tree = new octomap::OcTree(octree_resolution_);
  obj_tree_=tree;
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
  /*
  PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new PointCloud<pcl::PointXYZ>);
  _pcl_eigen.loadPLY(mesh_file_location,obj_cloud);

  
  _pcl_eigen.loadPCLOctree(obj_cloud,obj_tree_);
  */
  _pcl_eigen.loadBinvoxOctree(mesh_file_location,obj_tree_);
  // create collision boxes from octree:
  env_objs_=loadOctreeCollObj(obj_tree_);
  // store offsets from collision objects:
  store_offsets(env_objs_,obj_offsets_);
  cout<<"Object voxel boxes size: "<<env_objs_.size()<<endl;
  
}

void handEnv::update_grasp_obj_pose(vector<double> &pose)
{
  vector<Transform3d> box_poses;
  add_offsets(obj_offsets_,pose,box_poses);
  update_pose(env_objs_,box_poses);
  
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

void handEnv::check_contacts(vector<vector<double>> &l_poses, vector<vector<coll_data>> &c_data)
{
  //cerr<<"updating hand state"<<endl;
  // update robot link poses:
  
  update_hand_state(l_poses);
  
  c_data.resize(n_hand_links_);
  
  // get contact data
  for(int i=0;i<l_poses.size();++i)
  {
    signedDistance(hand_link_objs_[i], env_objs_,c_data[i]);
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
void handEnv::get_env_markers(visualization_msgs::MarkerArray &box_m, const string &frame_id)
{
  _pcl_eigen.get_marker_octree(obj_tree_,box_m,frame_id);

}
void handEnv::get_link_markers(visualization_msgs::MarkerArray &link_markers,const string &frame_id)
{
  // collision geometry does not contain the object information:
  link_markers.markers.resize(n_hand_links_);

  std_msgs::ColorRGBA m_color;

  m_color.r = 0.0;
  m_color.g = 0.0;
  m_color.b = 1.0;
  m_color.a = 1.0;
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  for(int i=0;i<n_hand_links_;++i)
  {
    const fcl::CollisionGeometry<double>* o_geom=hand_link_objs_[i]->getCollisionGeometry();
    // create sphere marker:
    Transform3d o_pose=hand_link_objs_[i]->getTransform();
    Quaterniond q_o(o_pose.rotation());
    Vector3d pos(o_pose.translation());
    Vector3d c=o_geom->aabb_center+pos;
    
    double r=o_geom->aabb_radius;

    //
    link_markers.markers[i].header.frame_id = frame_id;
    link_markers.markers[i].header.stamp = ros::Time();
    link_markers.markers[i].ns = frame_id;
    link_markers.markers[i].id = i;
    link_markers.markers[i].type = shape;
    
    link_markers.markers[i].scale.x = r;
    link_markers.markers[i].scale.y = r;
    link_markers.markers[i].scale.z = r;
    link_markers.markers[i].pose.position.x=c[0];
    link_markers.markers[i].pose.position.y=c[1];
    link_markers.markers[i].pose.position.z=c[2];
    
    link_markers.markers[i].pose.orientation.x=q_o.x();
    link_markers.markers[i].pose.orientation.y=q_o.y();
    link_markers.markers[i].pose.orientation.z=q_o.z();

    link_markers.markers[i].pose.orientation.w=q_o.w();

    link_markers.markers[i].color = m_color;

    //if (link_markers.markers[i].points.size() > 0)
    link_markers.markers[i].action = visualization_msgs::Marker::ADD;
    // else
    //  link_markers.markers[i].action = visualization_msgs::Marker::DELETE;

  }
}

}
