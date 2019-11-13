#include <in_hand_collision_lib/scene/hand_env.h>

namespace in_hand_manipulation
{

bool handEnv::update_robot_state(const vector<double> &q, const bool &arm_static)
{
  vector<double> robot_q;
  if(arm_static)
  {
    robot_q.insert(robot_q.end(),arm_q_.begin(),arm_q_.end());
  }
  robot_q.insert(robot_q.end(), q.begin(),q.end());
  // get link poses
  vector<vector<double>> r_poses;
  
  Eigen::Map<Eigen::VectorXd> q_eig(robot_q.data(), robot_q.size());

  robot_kdl_.getLinkPoses(q_eig,r_poses);
  // update link poses in collision library
  c_check.update_robot_state(r_poses);
  
  return true;
}

bool handEnv::update_arm_state(const vector<double> &q)
{
  arm_q_ = q;
}

void handEnv::get_ftip_contact_pt(vector<double> &in_pose,int ftip_idx, Vector3d &c_pt)
{
  /*
  Transform3d f_pose;
  add_offsets(hand_link_offsets_[ftip_idx],in_pose,f_pose);
  c_pt=f_pose.translation();
  */
}

void handEnv::check_ftip_contact(vector<double> &pose, coll_data &c_data, int ftip_idx)
{
  /*
  Transform3d f_pose;
  add_offsets(hand_link_offsets_[ftip_idx],pose,f_pose);
  update_pose(hand_link_objs_[ftip_idx],f_pose);
  
  // check collision data
  signedDistance(hand_link_objs_[ftip_idx],env_objs_,c_data);
  */
}


void handEnv::update_collision_links(const vector<int> &l_idx)
{
  // we load links against which collisions have to be checked in a seperate vector for easy collision checking:
  // thanks to pointers, we only store references, and when the link poses are updated, this list will also be updated.
  /*
  collision_link_objs_.resize(l_idx.size());
  for(int i=0;i<l_idx.size();++i)
  {
    collision_link_objs_[i]=hand_link_objs_[l_idx[i]];
  }
  */
  
}

void handEnv::check_link_collision(vector<vector<coll_data>> &c_data, vector<int> l_idx)
{
  //update_hand_state(l_poses);
  /*
  c_data.resize(n_hand_links_);
  for(int i=0;i<collision_link_objs_.size();++i)
  {
    signedDistance(collision_link_objs_[i],hand_link_objs_,c_data[i]);
  }
  */
}

void handEnv::check_link_collision( int l_idx, vector<double> &l_pose, coll_data &c_data)
{
  /*
  // update link state:
  update_link_state(l_pose,l_idx);

  signedDistance(hand_link_objs_[l_idx],env_objs_,c_data);
  */
}


void handEnv::check_contacts(vector<vector<double>> &l_poses, vector<vector<coll_data>> &c_data)
{
  //cerr<<"updating hand state"<<endl;
  // update robot link poses:
  /*
  update_hand_state(l_poses);
  
  c_data.resize(n_hand_links_);
  
  // get contact data
  for(int i=0;i<l_poses.size();++i)
  {
    signedDistance(hand_link_objs_[i], env_objs_,c_data[i]);
  }
  */
}
void handEnv::check_contacts(vector<vector<double>> &l_poses, vector<coll_data> &c_data)
{
  //cerr<<"updating hand state"<<endl;
  // update robot link poses:

  /*
  update_hand_state(l_poses);
  
  c_data.resize(n_hand_links_);
  
  // get contact data
  for(int i=0;i<l_poses.size();++i)
  {
    signedDistance(hand_link_objs_[i], env_objs_,c_data[i]);
  }
  */
}

void handEnv::check_contacts(vector<vector<double>> &l_poses, vector<vector<coll_data>> &c_data,vector<int> l_idx)
{
  /*
  //cerr<<"updating hand state"<<endl;
  // update robot link poses:
  
  update_hand_state(l_poses);
  
  c_data.resize(n_hand_links_);
  
  // get contact data
  for(int i=0;i<l_idx.size();++i)
  {
    signedDistance(hand_link_objs_[l_idx[i]], env_objs_,c_data[l_idx[i]]);
  }
  */
}
void handEnv::check_contacts(vector<vector<double>> &l_poses, vector<coll_data> &c_data,vector<int> l_idx)
{
  /*
  //cerr<<"updating hand state"<<endl;
  // update robot link poses:
  
  update_hand_state(l_poses);
  
  c_data.resize(n_hand_links_);
  
  // get contact data
  for(int i=0;i<l_idx.size();++i)
  {
    signedDistance(hand_link_objs_[l_idx[i]], env_objs_,c_data[l_idx[i]]);
  }
  */
}

/*
void handEnv::get_orient_contacts(vector<vector<double>> &l_poses, vector<int> l_idx, vector<Eigen::VectorXd> &or_grad,double delta)
{
  or_grad.resize(l_poses.size());
  // using central difference:
  
  for(int i=0;i<l_idx.size();++i)
  {
    int l=l_idx[i];
    // get link orientation in euler:
    Eigen::VectorXd l_orient=transforms.get_RPY_q(l_poses[l][4],l_poses[l][5],l_poses[l][6],l_poses[l][7]);
    Eigen::VectorXd l_or=l_orient;
    or_grad[i].resize(3);
    for(int j=0;j<3;++j)
    {
      l_or[j]+=delta;
      // update link pose:
      vector<double> l_pose(7,0.0);
      l_pose=l_poses[l];

      Eigen::VectorXd l_new=transforms.get_q_RPY(l_or[0],l_or[1],l_or[2]);
      l_pose[3]=l_new[0];
      l_pose[4]=l_new[1];
      l_pose[5]=l_new[2];
      l_pose[6]=l_new[3];
      
      // check contacts:
      coll_data c_dat;
      check_link_collision(l,l_pose,c_dat);
      double sd_p=c_dat.distance;
      l_or=l_orient;
      l_or[j]-=delta;
      
      l_new=transforms.get_q_RPY(l_or[0],l_or[1],l_or[2]);
      l_pose[3]=l_new[0];
      l_pose[4]=l_new[1];
      l_pose[5]=l_new[2];
      l_pose[6]=l_new[3];
   
      // check contacts:
      check_link_collision(l,l_pose,c_dat);
      
      double sd_n=c_dat.distance;
      or_grad[i][j]=(sd_p-sd_n)/(2*delta);
    }
  }
}
*/

/*
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
*/
}
