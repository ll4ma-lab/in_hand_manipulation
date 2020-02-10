// Finds the surface normal at the contact points and publishes them
#include <ll4ma_opt_utils/plan_env_utils.hpp>
#include <in_hand_manipulation_msgs/SurfaceNormal.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>


Eigen::Vector3d point;
string camera_frame;
void point_cb(geometry_msgs::PointStamped msg)
{
  point(0) = msg.point.x;
  point(1) = msg.point.y;
  point(2) = msg.point.z;
  //cout<<"msg:"<<point.transpose()<<endl;
  camera_frame = msg.header.frame_id;
}


visualization_msgs::Marker get_marker(const vector<double> &s_pt, const vector<double> &s_n,string frame_)
{
  visualization_msgs::Marker marker;
    
  marker.header.frame_id = frame_;
  marker.ns = "basic_shapes";
  
  //marker.id = i;
  
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  //marker.pose.orientation.w=1.0;  
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.type=0;
  geometry_msgs::Point start;
  start.x=s_pt[0];// mm
  start.y=s_pt[1];
  start.z=s_pt[2];
  
  marker.points.emplace_back(start);
  start.x+=0.1*s_n[0];
  start.y+=0.1*s_n[1];
  start.z+=0.1*s_n[2];
  marker.points.emplace_back(start);
  

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.0;
      
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1.0;
      
  marker.lifetime = ros::Duration();
    

  return marker;
}
int main(int argc, char** argv)
{
  point.setZero();
  camera_frame = "world";
  ros::init(argc,argv,"in_hand_utils/point_surface_normal_node");

  ros::NodeHandle n;
  Eigen::Vector3d surface_normal,raw_sn;
  Eigen::Vector3d old_surface_normal;

  // subscribe to robot joint state
  std::string  obj_frame;
  //n.getParam("robot_joint_state",js_topic);
  n.getParam("object_frame",obj_frame);
  // get gravity with respect to world:
  vector<double> w_g;
  n.getParam("w_gravity",w_g);
  
  w_g[0] = -1.0 * w_g[0];
  w_g[1] = -1.0 * w_g[1];
  w_g[2] = -1.0 * w_g[2];
  
  Eigen::Vector3d g_f = Eigen::Map<Eigen::Vector3d>(w_g.data(),w_g.size());
  cout<<g_f.transpose()<<endl;
  //ROS_INFO("JS topic");
    
  ros::Subscriber point_sub=n.subscribe("/biotac_detector/point",1,point_cb);
  ros::Publisher m_pub=n.advertise<visualization_msgs::MarkerArray>("/in_hand_utils/surface_normal",1);
  // initialize environment
  ll4ma_opt_utils::envUtils env_utils;
  env_utils.init(n);
  //env_utils.init_robot_meshes();
  env_utils.init_obj();
  surface_normal.setZero();
  old_surface_normal.setZero();
  raw_sn.setZero();
  vector<double> cpts;
  //cpts.resize(3);
  // read object pose from tf:
  tf::TransformListener listener;

  ros::Rate rate(10);

  //vector<string> cpt_frames;
  //bool tactile_sensing=n.getParam("cpt_frames",cpt_frames);
  
  visualization_msgs::MarkerArray m_arr;
      
  
  while(n.ok())
  {
    //cout<<point.transpose()<<endl;
    ros::spinOnce();
    rate.sleep();
        
    tf::StampedTransform transform;
    try{
      //ROS_INFO("Trying..");
      listener.lookupTransform(camera_frame,obj_frame,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    tf::Vector3 origin=transform.getOrigin();
    tf::Quaternion q=transform.getRotation();
    vector<double> pose;
    pose.resize(7);
    pose[0]=origin.x();
    pose[1]=origin.y();
    pose[2]=origin.z();

    pose[3]=q.x();
    pose[4]=q.y();
    pose[5]=q.z();
    pose[6]=q.w();
    //
    env_utils.base_frame_ = camera_frame;

    env_utils.update_obj_pose(pose);

    // read tactile contact points if available:
    if(point.lpNorm<1>() > 0.0)
    {
      m_arr.markers.clear();
      vector<double> c_pt={point(0),point(1),point(2)};
      vector<double> s_pt,s_n;
      env_utils.c_check.obj_surface_normal(c_pt,s_pt,s_n);
      // get surface point
      // visualize as arrow

      raw_sn=Eigen::Map<Eigen::VectorXd>(s_n.data(),s_n.size());
      surface_normal=0.6*surface_normal+0.198*old_surface_normal+0.198*raw_sn;
      old_surface_normal=raw_sn;
      vector<double> s_n_filtered;
      s_n_filtered.resize(3);
      Eigen::Vector3d::Map(&s_n_filtered[0],3)=surface_normal;
      visualization_msgs::Marker m=get_marker(s_pt,s_n,camera_frame);

      // get angle between s_n and g:
      cout<<tan(acos(raw_sn.dot(g_f)))<<endl;
      //cout<<surface_normal.dot(g_f)<<endl;

      m.id=0;
      m_arr.markers.emplace_back(m);
      m=get_marker(s_pt,w_g,camera_frame);
      m.color.g = 1.0;
      m.id=1;
      m_arr.markers.emplace_back(m);

      m_pub.publish(m_arr);

    }
    // publish contact point on object surface with surface normal:
  }
  
  
  
  return 0;
  
}
