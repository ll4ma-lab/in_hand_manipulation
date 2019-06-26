// Finds the surface normal at the contact points and publishes them
#include <ll4ma_opt_utils/plan_env_utils.hpp>
#include <in_hand_manipulation_msgs/SurfaceNormal.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>


Eigen::VectorXd js;

void js_cb(sensor_msgs::JointState msg)
{
  js=Eigen::Map<Eigen::VectorXd>(msg.position.data(),msg.position.size());
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
  ros::init(argc,argv,"in_hand_utils/surface_normal_node");

  ros::NodeHandle n;
  vector<Eigen::Vector3d> surface_normal,raw_sn;
  vector<Eigen::Vector3d> old_surface_normal;

  // subscribe to robot joint state
  std::string js_topic, obj_frame;
  n.getParam("robot_joint_state",js_topic);
  n.getParam("object_frame",obj_frame);
  ROS_INFO("JS topic");
    
  ros::Subscriber js_sub=n.subscribe(js_topic,1,js_cb);
  ros::Publisher m_pub=n.advertise<visualization_msgs::MarkerArray>("/in_hand_utils/surface_normal",1);
  // initialize environment
  ll4ma_opt_utils::envUtils env_utils;
  env_utils.init(n);
  env_utils.init_robot_meshes();
  env_utils.init_obj();
  surface_normal.resize(env_utils.c_check.n_hand_links_,Eigen::Vector3d::Zero());
  old_surface_normal.resize(env_utils.c_check.n_hand_links_,Eigen::Vector3d::Zero());
  raw_sn.resize(env_utils.c_check.n_hand_links_,Eigen::Vector3d::Zero());
  // read object pose from tf:
  tf::TransformListener listener;

  ros::Rate rate(10);

  while(n.ok())
  {
    ros::spinOnce();
    rate.sleep();
        
    tf::StampedTransform transform;
    try{
      //ROS_INFO("Trying..");
      listener.lookupTransform(env_utils.base_frame_,obj_frame,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    if(js.size()<1)
    {
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
    env_utils.c_check.update_obj_pose(pose);
    
    // get contact points between hand links and object
    
    vector<vector<double>> r_poses;
    env_utils.robot_kdl_.getLinkPoses(js,r_poses);
    //
    env_utils.c_check.update_robot_state(r_poses);
    vector<coll_data> c_data;
    env_utils.c_check.hand_object_ccheck(c_data,true);
    visualization_msgs::MarkerArray m_arr;

    for(int i=0;i<c_data.size();++i)
    {
      vector<double> c_pt={c_data[i].p2[0],c_data[i].p2[1],c_data[i].p2[2]};// object point
      vector<double> c_pt1={c_data[i].p1[0],c_data[i].p1[1],c_data[i].p1[2]};// robot point

      vector<double> s_pt,s_n;
      env_utils.c_check.obj_surface_normal(c_pt,s_pt,s_n);
      // get surface point
      // visualize as arrow

      raw_sn[i]=Eigen::Map<Eigen::VectorXd>(s_n.data(),s_n.size());
      surface_normal[i]=0.6*surface_normal[i]+0.198*old_surface_normal[i]+0.198*raw_sn[i];
      old_surface_normal[i]=raw_sn[i];
      vector<double> s_n_filtered;
      s_n_filtered.resize(3);
      Eigen::Vector3d::Map(&s_n_filtered[0],3)=surface_normal[i];
      visualization_msgs::Marker m=get_marker(s_pt,s_n_filtered,env_utils.base_frame_);
      m.id=i;
      m_arr.markers.emplace_back(m);

    }
    m_pub.publish(m_arr);
    // publish contact point on object surface with surface normal:
    
  }
  
  
  return 0;
  
}
