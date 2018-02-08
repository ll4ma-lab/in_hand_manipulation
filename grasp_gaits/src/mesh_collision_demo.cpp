#include <grasp_gaits/UpdateObj.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <grasp_gaits/scene/allegro_env.h>
#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>
int main(int argc,char** argv)
{
  ros::init(argc,argv,"mesh_collision_demo");
  ros::NodeHandle n;
  ros::Publisher pub;
  pub=n.advertise<pcl::PointCloud<PointXYZRGB>>("/collision_checker/object",1);
  ros::Publisher pub_1=n.advertise<pcl::PointCloud<PointXYZRGB>>("/collision_checker/des_object",1);

  // create collision object:
  allegroEnv c_check(0.1,0.1);

  // load finger mesh:
  std::string hand_mesh_location=ros::package::getPath("urlg_robots_description");
  hand_mesh_location.append("/meshes/allegro/convex_hull/vertices/");
  vector<string> hand_link_names={"index_link_1","index_link_2","index_tip", "index_link_1","index_link_2","index_tip","index_link_1","index_link_2","index_tip", "thumb_link_2","thumb_link_3","thumb_tip"};

  // load hand links:
  for(int i=0;i<hand_link_names.size();++i)
  {
    hand_link_names[i].insert(0,hand_mesh_location);
    hand_link_names[i].append(".ply");
  }
  c_check.load_hand_links(hand_link_names);

  // load environment:
  string mesh="/home/bala/object_models/mustard_bottle/google_16k/vertices.ply";
  vector<double> pose(7,0.0);
  pose[6]=1.0;
  pose[0]=0.4;
  pose[1]=0.1;
  pose[2]=0.3;
    
  // load obj mesh
  c_check.load_obj(mesh);

  // update object pose
  c_check.update_grasp_obj_pose(pose);


  double x=atof(argv[1]);
  double y=atof(argv[2]);
  double z=atof(argv[3]);
  
  // update link object pose:
  vector<collision_data> c_data;
  vector<double> ftip_pose={x,y,z,0.0,0.0,0.0,1.0};

  ros::Rate loop_r(5);
  while(ros::ok())
  {
    c_check.check_ftip_contact(ftip_pose,c_data,2);
    
    // get point cloud
    pcl::PointCloud<PointXYZRGB>::Ptr cloud_obj1(new PointCloud<PointXYZRGB>);
    c_check.get_link_pointcloud(cloud_obj1,2);
    c_check.get_obj_pointcloud(cloud_obj1);
    pcl::PointCloud<PointXYZRGB>::Ptr cloud_obj2(new PointCloud<PointXYZRGB>);
    vector<double> new_pose=ftip_pose;
    Eigen::VectorXd sd_contact(c_data.size());
    sd_contact.setZero();

    
    for(int k=0;k<c_data.size();++k)
    {
      sd_contact[k]=std::abs(c_data[k].distance);
    }
    int min_idx;
    sd_contact.minCoeff(&min_idx);
    
    Map<Eigen::VectorXd> point(&c_data[min_idx].point[0],3);
    Map<Eigen::VectorXd> dir(&c_data[min_idx].dir[0],3);
    double p_d=c_data[min_idx].distance;
    Eigen::VectorXd delta_p(3);

    cerr<<p_d<<endl;


    // testing collision avoidance gradient:
    if(p_d>-0.01 && p_d<0.0)// when mesh is close to contact with the object
    {
      cerr<<"d"<<endl;
      delta_p=-(p_d)*dir;
    }
    else if(p_d>0.0)
    {
      delta_p=(p_d+0.02)*dir;
    }
    else// this will move the mesh close to the object, this can be used to make contact with the object
    {
      delta_p=(p_d+0.02)*dir;
      //delta_p.setZero();
    }

    if(!std::isnan(delta_p[0]))
    {
      new_pose[0]+=delta_p[0];
      new_pose[1]+=delta_p[1];
      new_pose[2]+=delta_p[2];
    }
    cerr<<delta_p.transpose()<<endl;
    c_check.check_ftip_contact(new_pose,c_data,2);
    c_check.get_link_pointcloud(cloud_obj2,2);
    for(int i=0;i<4;++i)
    {
      pub_1.publish(cloud_obj2);
      pub.publish(cloud_obj1);
      loop_r.sleep();
    }
    ftip_pose=new_pose;
    getchar();
    //cin<<"h";
  }
  
  //ros::spin();// Loop rate?
}
