// This demo loads the allegro hand urdf from rosparam, 

#include <ros/ros.h>
#include <ros/package.h>

//#include <in_hand_manipulation_msgs/UpdateObj.h>
#include <in_hand_collision_lib/scene/hand_env.h>

//#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>

#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

using namespace in_hand_manipulation;
class srvClass
{
public:
  handEnv hand_env;
  ros::NodeHandle* nh;
  ros::Publisher pub,m_pub,l_pub,c_pub;
  manipulator_kdl::robotKDL hand_kdl;
  bool collision_ready=false;
  
  void init(ros::NodeHandle &n)
  {
    nh=&n;
    
    handEnv rob_env_checker(n);
    hand_env=rob_env_checker;
  }
  
  bool add_obj_call(ll4ma_opt_utils::AddObject::Request &req, ll4ma_opt_utils::AddObject::Response &res)
  {
    return hand_env.load_obj_call(req,res);
  }
  
  bool init_robot_meshes()
  {
    c_pub=nh->advertise<visualization_msgs::Marker>("contact_pt", 1);

    
    hand_env.load_robot_model();
    hand_env.load_obj();
    collision_ready = true;
    return true;
    /*
    l_pub=nh->advertise<visualization_msgs::MarkerArray>("link_env", 1);
    
    vector<double> g_vec={0.0,0.0,-9.8};

    vector<string> hand_link_names={"index_link_1","index_link_2","index_tip", "middle_link_1","middle_tip","middle_tip","ring_link_1","ring_link_2","ring_tip", "thumb_link_2","thumb_link_3","thumb_tip"};

    // initialize collision environment:
    std::string robot_desc_string;
    
    // read urdf as a string:
    nh->param("robot_description", robot_desc_string, std::string());
    hand_env.load_hand_links(hand_link_names,robot_desc_string);


    vector<string> hand_base_names={"palm_link","palm_link","palm_link","palm_link"};
    vector<string> hand_ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"}; 

manipulator_kdl::robotKDL allegro_kdl("robot_description",*nh,hand_base_names, hand_ee_names, hand_link_names,g_vec);
    hand_kdl=allegro_kdl;
    
    */
  }
  void jCallback(sensor_msgs::JointState j_msg)
  {
    vector<double> hand_js = j_msg.position;
    if(collision_ready)
    {
      check_hand_coll(hand_js);
    }

  }
  void check_hand_coll(vector<double> js)
  {
    //cerr<<"checing collisions.."<<endl;

    // get link poses:
    vector<double> arm_js(7,0.0);
    
    hand_env.update_arm_state(arm_js);
    
    hand_env.update_robot_state(js,false);
    // check collision between hand links
    vector<coll_data> hand_c_data;
    
    hand_env.c_check.hand_object_ccheck(hand_c_data);

    visualization_msgs::Marker c_pt;
    std_msgs::ColorRGBA m_color;
    
    m_color.r = 1.0;
    m_color.g = 0.0;
    m_color.b = 0.0;
    m_color.a = 1.0;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    c_pt.header.frame_id=hand_env.base_frame_;
    c_pt.header.stamp=ros::Time();
    c_pt.id=0;
    c_pt.type=shape;
    c_pt.scale.x=0.01;
    c_pt.scale.y=0.01;
    c_pt.scale.z=0.01;
    c_pt.pose.orientation.w=1.0;

    c_pt.pose.position.x=0.0;
    c_pt.pose.position.y=0.0;
    c_pt.pose.position.z=0.0;
    c_pt.color=m_color;

    if(hand_c_data.size()>0)
    {
      for(int i=0;i<hand_c_data.size();++i)
      {
        if(hand_c_data[i].distance<0.0)
        {
          cerr<<"hand link in collision: "<<i<<" d: "<<hand_c_data[i].distance<<endl;
          cerr<<hand_c_data[i].p1.transpose()<<endl;
          cerr<<hand_c_data[i].p2.transpose()<<endl;
          c_pt.pose.position.x=hand_c_data[i].p2(0);
          c_pt.pose.position.y=hand_c_data[i].p2(1);
          c_pt.pose.position.z=hand_c_data[i].p2(2);
          
	}
      }
    }
    ros::Rate l(100);

    for(int i=0;i<10;++i)
    {
      c_pub.publish(c_pt);
      l.sleep();
    }
    /*
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

    visualization_msgs::Marker c_pt;
    std_msgs::ColorRGBA m_color;
    
    m_color.r = 1.0;
    m_color.g = 0.0;
    m_color.b = 0.0;
    m_color.a = 1.0;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    c_pt.header.frame_id="palm_link";
    c_pt.header.stamp=ros::Time();
    c_pt.id=0;
    c_pt.type=shape;
    c_pt.scale.x=0.01;
    c_pt.scale.y=0.01;
    c_pt.scale.z=0.01;
    c_pt.pose.orientation.w=1.0;

    c_pt.pose.position.x=0.0;
    c_pt.pose.position.y=0.0;
    c_pt.pose.position.z=0.0;
    c_pt.color=m_color;
    for(int i=0;i<10;++i)
    {
      l_pub.publish(oc_map);
    }
    
    
    */
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"allegro_contact_demo");
  ros::NodeHandle n;
  srvClass srv_call;
  srv_call.init(n);
  srv_call.init_robot_meshes();
  // update object model:
  ros::ServiceServer service_obj=n.advertiseService("/in_hand_manipulation/update_object", &srvClass::add_obj_call, &srv_call);

  ros::Subscriber jsub=n.subscribe("/lbr4_allegro/joint_states",1,&srvClass::jCallback,&srv_call);
  
  ros::spin();
  
}
