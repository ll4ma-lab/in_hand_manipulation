#include <grasp_gaits/scene/allegro_env.h>
#include <pcl_mesh_tools/libs/collision_checker/support_fn.h>
// constructor:
allegroEnv::allegroEnv()
{
}
allegroEnv::allegroEnv(float narrow_vsize, float broad_vsize)
{
  narrow_vsize_=narrow_vsize;
  broad_vsize_=broad_vsize;
  CCD_INIT(&ccd); // initialize ccd_t struct
  
  ccd.support1       = support_convex; // support function for first object
  ccd.support2       = support_convex; // support function for second object
  ccd.max_iterations = 10;     // maximal number of iterations
  ccd.epa_tolerance  = 0.005;  // maximal tolerance for EPA part

}

void allegroEnv::load_hand_links(vector<string> &f_names)
{
  n_hand_links_=f_names.size();

  load_meshes(f_names, hand_link_meshes_);
  create_coll_objects(hand_link_meshes_,hand_link_objs_);
}

void allegroEnv::update_hand_state(vector<vector<double>> &link_poses)
{
  update_poses(hand_link_objs_,link_poses);
}

void allegroEnv::load_obj(const string &mesh_file_location)
{
  PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new PointCloud<pcl::PointXYZ>);
  _pcl_eigen.loadPLY(mesh_file_location,obj_cloud);

  // HACD on object pointcloud:
  load_PCL(obj_cloud,narrow_vsize_,obj_meshes_);
  create_coll_objects(obj_meshes_,obj_);
  cerr<<"Obj size: "<<obj_.size()<<endl;
  
}
void allegroEnv::update_grasp_obj_pose(vector<double> &pose)
{
  update_poses(obj_,pose);

}



void allegroEnv::check_contacts(vector<vector<double>> &l_poses, vector<vector<collision_data>> &c_data,vector<int> l_idx)
{
  //cerr<<"updating hand state"<<endl;
  // update robot link poses:
  update_hand_state(l_poses);
  //cerr<<"updated hand state"<<endl;
  c_data.resize(n_hand_links_);
  
  // get contact data
  multi_threaded_contact(hand_link_objs_,obj_,l_idx,ccd,c_data);
  //single_contact(hand_link_objs_,obj_,l_idx,ccd,c_data);
}

vector<collision_data> allegroEnv::contact_fn(const obj_t &link_obj, const vector<obj_t> &narrow_env_objs, const ccd_t &ccd) const
{
  vector<collision_data> c_data;
  signed_distance_arr(link_obj,narrow_env_objs,ccd,c_data);
  return c_data;
}



void allegroEnv::single_contact(const vector<obj_t> &link_objs, const vector<obj_t> &env_objs, vector<int> l_idx, const ccd_t &ccd,vector<vector<collision_data>> &c_data) const
{
  for( int i=0;i<l_idx.size();++i)
  {
    c_data[l_idx[i]]=contact_fn(link_objs[l_idx[i]], env_objs,ccd);
  }
}

void allegroEnv::multi_threaded_contact(const vector<obj_t> &link_objs, const vector<obj_t> &env_objs, vector<int> l_idx, const ccd_t &ccd,vector<vector<collision_data>> &c_data) const
{
  vector<std::future<vector<collision_data>>> futures;

  for( int i=0;i<l_idx.size();++i)
  {
    futures.push_back(std::async(launch::async, &allegroEnv::contact_fn, this, link_objs[l_idx[i]], env_objs,ccd));
  }

  for (int i=0;i<futures.size();++i)
  {
    c_data[l_idx[i]]=futures[i].get();
  }
  
}


void allegroEnv::multi_threaded(const vector<obj_t> &link_objs, const vector<obj_t> &broad_env_objs, const vector<obj_t> &narrow_env_objs,const ccd_t &ccd,vector<vector<collision_data>> &c_data) const
{
  // parallel the below loop:
  // each thread i stores: link_objs[i],broad_env_objs_, narrow_env_objs=c_data[i]
  vector<std::future<vector<collision_data>>> futures;
  
  for( int i=0;i<n_links_;++i)
  {
    futures.push_back(std::async(launch::async, &allegroEnv::multi_thrd_fn, this, link_objs[i], broad_env_objs,narrow_env_objs,ccd));
  }
  
  for (int i=0;i<n_links_;++i)
  {
    c_data[i]=futures[i].get();
  }
}


vector<collision_data> allegroEnv::multi_thrd_fn(const obj_t &link_obj, const vector<obj_t> &broad_env_objs, const vector<obj_t> &narrow_env_objs, const ccd_t &ccd) const
{
  vector<collision_data> c_data;
  
  bool collision=false;
  detect_collisions(link_obj,broad_env_objs,ccd,collision);
  if(collision==true)
  {
    signed_distance_arr(link_obj,narrow_env_objs,ccd,c_data);
  }
  return c_data;
}

