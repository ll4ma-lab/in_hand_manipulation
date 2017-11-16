#include "in_grasp_manipulation/in_grasp_sim.h"
#include <tf/transform_broadcaster.h>
#include <random>

namespace in_grasp_sim
{
void robotSim::init(ros::NodeHandle& n)
{
  cmd_sub=n.subscribe<sensor_msgs::JointState>("allegro_hand_right/joint_cmd",1,&in_grasp_sim::robotSim::jntCallback,this);
 j_pub=n.advertise<sensor_msgs::JointState>("allegro_hand_right/joint_states",1);
}
// setup ros service to setup object transform
void robotSim::update_robot_state(sensor_msgs::JointState j_cmd)
{
  curr_js_=j_cmd;
  curr_js_.header.stamp=ros::Time::now();
}
void robotSim::jntCallback(sensor_msgs::JointState j)

{
  j_cmd_=j;
  update_robot_state(j_cmd_);
  new_jcmd=true;
}

void inGraspSim::update_in_grasp_state()
{
  // get thumb pose
  vector<double> th_joints;
  for(int i=0;i<4;++i)
  {
    th_joints.emplace_back(curr_js_.position[r_f*4+i]);
  }
  Eigen::MatrixXd p_T_th=hand_kdl.getFKEigen(r_f,th_joints);
   if(new_jcmd)
  {
    for(int l=0;l<4;++l)
    {
      for(int k=0;k<3;++k)
      {
        noise_mat(k,l)=dist(generator);
        if(l<3)
        {
          noise_mat(k,l)*=1.0;
        }

      }
    }
    new_jcmd=false;
    //cerr<<noise_mat<<endl;
    th_T_obj=initial_T_obj+noise_mat;
  }


  
  // get object pose
  Eigen::MatrixXd mat=p_T_th*th_T_obj;
  // Add noise:
 
  vector<double> obj_pose=hand_kdl.frame_to_pose_(mat);
  // update object pose and publish:
  obj_pose_msg.pose.position.x=obj_pose[0];
  obj_pose_msg.pose.position.y=obj_pose[1];
  obj_pose_msg.pose.position.z=obj_pose[2];
  obj_pose_msg.pose.orientation.x=obj_pose[3];
  obj_pose_msg.pose.orientation.y=obj_pose[4];
  obj_pose_msg.pose.orientation.z=obj_pose[5];
  obj_pose_msg.pose.orientation.w=obj_pose[6];
  obj_pose_msg.header.frame_id="palm_link";
  //obj_pub.publish(obj_pose_msg);
}
bool inGraspSim::setup_object(in_grasp_manipulation::InGraspSim::Request& req, in_grasp_manipulation::InGraspSim::Response& res)
{
  noise_mat.setZero();
  
  // Get thumb pose
  r_f=req.rigid_finger;
  vector<double> th_joints;
  for(int i=0;i<4;++i)
  {
    th_joints.emplace_back(req.init_joints.position[r_f*4+i]);
  }
  Eigen::MatrixXd p_T_th=hand_kdl.getFKEigen(r_f,th_joints);
  
  // Get initial object to thumb frame:
  tf::Pose init_pose;
  tf::poseMsgToTF(req.object_pose,init_pose);
  // init_pose->4x4
  tf::Quaternion q(req.object_pose.orientation.x,req.object_pose.orientation.y,req.object_pose.orientation.z,req.object_pose.orientation.w);
  tf::Matrix3x3 R(q);
  
  Eigen::MatrixXd obj_mat(4,4);
  obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    obj_mat(i,0)=R[i][0];
    obj_mat(i,1)=R[i][1];
    obj_mat(i,2)=R[i][2];
  }
  obj_mat(0,3)=req.object_pose.position.x;
  obj_mat(1,3)=req.object_pose.position.y;
  obj_mat(2,3)=req.object_pose.position.z;

  // Find thumb to object frame transformation
  th_T_obj=p_T_th.inverse()*obj_mat;
  initial_T_obj=th_T_obj;
  setup_obj_frame=true;
  const double mean = 0.0;
  const double stddev = 0.001;
  std::normal_distribution<double> dist_(mean, stddev);
  dist=dist_;

}
void inGraspSim::init_service(ros::NodeHandle& n)
{
  in_grasp_init_service=n.advertiseService("/in_grasp_sim/init",&in_grasp_sim::inGraspSim::setup_object,this);
  // build hand kdl
    string urdf_file="/home/bala/catkin_ws/src/in_hand_manipulation/in_grasp_manipulation/urdf/allegro.urdf";

  // Load allegro kdl object:
  vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
  vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics

  manipulator_kdl::robotKDL allegro_kdl(urdf_file,base_names,ee_names,g_vec);
  allegro_kdl.urdfParser();
  hand_kdl=allegro_kdl;
  obj_pub=n.advertise<geometry_msgs::PoseStamped>("/obj_pose",1);
}
void inGraspSim::publish_state()
{
  if(setup_obj_frame)
  {
    obj_pose_msg.header.stamp=ros::Time::now();
    obj_pub.publish(obj_pose_msg);
    // publish tf:
    vector<double> rigid_frame= hand_kdl.frame_to_pose_(th_T_obj);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(rigid_frame[0],rigid_frame[1], rigid_frame[2]) );
    tf::Quaternion q(rigid_frame[3],rigid_frame[4],rigid_frame[5],rigid_frame[6]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "thumb_tip","/dbot/obj_pose"));
  }
  
  curr_js_.header.stamp=ros::Time::now();
  j_pub.publish(curr_js_);
  
}
}
