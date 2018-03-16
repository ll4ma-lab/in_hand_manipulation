// This demo loads the allegro hand urdf from rosparam, 

#include <ros/ros.h>
#include <ros/package.h>

#include <in_hand_manipulation_msgs/UpdateObj.h>
#include <in_hand_collision_lib/scene/hand_env.h>

#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>

#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

using namespace fcl_ccheck;
class srvClass
{
public:
  handEnv hand_env;
  ros::NodeHandle* nh;
  ros::Publisher pub,m_pub,l_pub;
  manipulator_kdl::robotKDL hand_kdl;
  vector<vector<coll_data>> hand_c_data;
  bool collision_ready=false;
  
  void init(ros::NodeHandle &n)
  {
    nh=&n;
    
    handEnv rob_env_checker(0.01);
    hand_env=rob_env_checker;
    //cerr<<"copied rob_env_scene"<<endl;
  }
  void init_robot_meshes()
  {
    l_pub=nh->advertise<visualization_msgs::MarkerArray>("link_env", 1);

    vector<double> g_vec={0.0,0.0,-9.8};

    vector<string> hand_link_names={"index_link_1","index_link_2","index_tip", "middle_link_1","middle_link_2","middle_tip","ring_link_1","ring_link_2","ring_tip", "thumb_link_2","thumb_link_3","thumb_tip"};

    // initialize collision environment:
    std::string robot_desc_string;
    
    // read urdf as a string:
    nh->param("robot_description", robot_desc_string, std::string());
    hand_env.load_hand_links(hand_link_names,robot_desc_string);


    vector<string> hand_base_names={"palm_link","palm_link","palm_link","palm_link"};
    vector<string> hand_ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"}; 

manipulator_kdl::robotKDL allegro_kdl("robot_description",*nh,hand_base_names, hand_ee_names, hand_link_names,g_vec);
    hand_kdl=allegro_kdl;
    
      
    cerr<<"Created robot collision files"<<endl;
  }
  bool update_obj(in_hand_manipulation_msgs::UpdateObj::Request &req, in_hand_manipulation_msgs::UpdateObj::Response &res)
  {
    //pub=nh->advertise<pcl::PointCloud<PointXYZ>>("/collision_checker/env",1);
    m_pub=nh->advertise<visualization_msgs::MarkerArray>("collision_env", 1);
    ros::Rate l(100);
cerr<<"Updating environment cloud"<<endl;
    //convert sensor_msgs to normal pointcloud

    hand_env.load_obj(req.mesh_file);
    //
    vector<double> pose(7,0.0);
    pose[0]=req.obj_pose.position.x;
    pose[1]=req.obj_pose.position.y;
    pose[2]=req.obj_pose.position.z;
    pose[3]=req.obj_pose.orientation.x;
    pose[4]=req.obj_pose.orientation.y;
    pose[5]=req.obj_pose.orientation.z;
    pose[6]=req.obj_pose.orientation.w;
    
                       
    hand_env.update_grasp_obj_pose(pose);
    //cerr<<pose[0]<<endl;
    // publish tf:
    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose[0],pose[1],pose[2]) );
    tf::Quaternion q(pose[3],pose[4],pose[5],pose[6]);
    transform.setRotation(q);

    visualization_msgs::MarkerArray oc_map; 
    hand_env.get_env_markers(oc_map,"obj_frame");

    // visualize pointcloud:
    for(int i=0;i<100;++i)
    {
       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "palm_link", "obj_frame"));

      // visualize octomap:
      m_pub.publish(oc_map);
      l.sleep();
    }
    collision_ready=true;
    return true;
  }
  void jCallback(sensor_msgs::JointState j_msg)
  {
    Eigen::VectorXd hand_js;
    hand_js.resize(16);

    for(int i=0;i<16;++i)
    {
      hand_js[i]=j_msg.position[i];
    }
    if(collision_ready)
    {
      check_hand_coll(hand_js);
    }

  }
  void check_hand_coll(Eigen::VectorXd js)
  {
    // get link poses:

    vector<vector<double>> r_poses;
    r_poses.resize(4*3);
    Eigen::VectorXd hand_q(4);
    Eigen::VectorXd l_pose_;
    

    for(int i=0;i<4;++i)
    {
      for(int j=0;j<3;++j)
      {
        hand_q.tail(4)=js.segment(i*4,4);
        hand_kdl.getLinkPose(i*3+j,hand_q,l_pose_);
        hand_kdl.getSTDVector(l_pose_,r_poses[i*3+j]);
      }
    }
    //cerr<<"checking for collisions"<<endl;
    // check for collisions:
    hand_env.check_contacts(r_poses,hand_c_data);
    visualization_msgs::MarkerArray oc_map; 
    hand_env.get_link_markers(oc_map,"palm_link");

    for(int i=0;i<10;++i)
    {
      l_pub.publish(oc_map);
    }

    if(hand_c_data.size()>0)
    {
      for(int i=0;i<hand_c_data.size();++i)
      {
	for(int j=0;j<hand_c_data[i].size();++j)
	{
          //cerr<<"hand link in collision: "<<i<<" d: "<<hand_c_data[i][j].distance<<endl;
	  if(hand_c_data[i][j].distance<0.0)
	  {
	    cerr<<"hand link in collision: "<<i<<" d: "<<hand_c_data[i][j].distance<<endl;
	  }
	}
      }
    }

  }

};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"allegro_contact_demo");
  ros::NodeHandle n;
  srvClass srv_call;
  srv_call.init(n);
  srv_call.init_robot_meshes();
  // update environment server:
  ros::ServiceServer env_service=n.advertiseService( "/update_obj", &srvClass::update_obj, &srv_call);

  ros::Subscriber jsub=n.subscribe("/joint_states",1,&srvClass::jCallback,&srv_call);
  ros::spin();
  
}
