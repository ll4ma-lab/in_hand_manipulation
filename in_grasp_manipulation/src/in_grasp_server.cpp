#define SPARSE_GRADIENT true
// optimization:
#include <in_grasp_manipulation/problems/in_grasp_opt_problem.hpp>
#include <ll4ma_opt_wrapper/pagmo_opt/wrapper.hpp>

//#include <ll4ma_planner/wrapper/pagmo_wrapper.hpp>
// ROS:
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include <Eigen/Geometry> 

// ROS service for in grasp manipulation:
#include "in_grasp_manipulation/InGraspManipulation.h"

// joint trajectory
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

using namespace pagmo;
using namespace Eigen;


class srvClass
{
public:
  //manipulationEnv
  ros::NodeHandle* nh;
  ros::Publisher pub,m_pub;
  string base_frame_;
  manipulator_kdl::robotKDL robot_kdl_;
  vector<int> contact_links;
  vector<double> obj_pose;
  void pub_marker(string uri_file_name, vector<double> pose, string pub_topic,vector<double> color={0.8,0.8,0.8,1.0})
  {
    m_pub=nh->advertise<visualization_msgs::Marker>(pub_topic, 1);

    visualization_msgs::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.ns = "basic_shapes";
    marker.id = 0;
  
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pose[0];
    marker.pose.position.y = pose[1];
    marker.pose.position.z = pose[2];
    // compute quaternion from rpy:
    tf::Quaternion q;
    q.setRPY(pose[3],pose[4],pose[5]);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1;
    marker.scale.z = 1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    marker.lifetime = ros::Duration();
    

  // mesh marker:
    uint32_t mesh_shape= visualization_msgs::Marker::MESH_RESOURCE;
    marker.type=mesh_shape;
    marker.mesh_resource=uri_file_name;
    ros::Rate l(100);

    for(int i=0;i<100;++i)
    {
      m_pub.publish(marker);
      l.sleep();
    }

  }
  void init(ros::NodeHandle &n)
  {
    nh=&n;
    /*
    cerr<<"initializing rob_env_scene"<<endl;
    manipulationEnv rob_env_checker;
    c_check=rob_env_checker;
    cerr<<"copied rob_env_scene"<<endl;
    */
  }
  /*
  void init_env()
  {
    // load environment mesh:
    string env_mesh;
    assert(nh->getParam("env_mesh",env_mesh));
    vector<double> pose;
    assert(nh->getParam("env_pose",pose));
    
    string stl_file=ros_uri::absolute_path(env_mesh);


    
    vector<string> f_names={stl_file};
    
    vector<Eigen::Matrix4d> t_;
    t_.resize(1);
    t_[0].setIdentity();
    
    t_[0](0,3)=pose[0];
    t_[0](1,3)=pose[1];
    t_[0](2,3)=pose[2];

    // compute rotation matrix from euler:
    tf::Matrix3x3 R;
    R.setRPY(pose[3],pose[4],pose[5]);
    for (int i=0;i<3;++i)
    {
      t_[0](i,0)=R[i].x();
      t_[0](i,1)=R[i].y();
      t_[0](i,2)=R[i].z();

    }
    

    c_check.add_env_meshes(f_names,t_);
    cerr<<"added environment mesh!"<<endl;
    // publish environment as marker:
    pub_marker(env_mesh,pose,"env_marker");

    // load reachability mesh:
    
    string reach_mesh;
    assert(nh->getParam("reach_mesh",reach_mesh));
    vector<double> reach_pose;
    assert(nh->getParam("reach_pose",reach_pose));
    vector<double> colr={1.0,0.0,0.0,0.2};
    pub_marker(reach_mesh,reach_pose,"reach_marker",colr);
    
    

  }
  */
  /*
  void init_obj()
  {

    string obj_mesh;
    assert(nh->getParam("obj_mesh",obj_mesh));
    vector<double> pose;
    assert(nh->getParam("obj_pose",pose));
    obj_pose=pose;
    string stl_file=ros_uri::absolute_path(obj_mesh);

    
    Eigen::Matrix4d t_;
    t_.setIdentity();
    
    t_(0,3)=pose[0];
    t_(1,3)=pose[1];
    t_(2,3)=pose[2];

    // compute rotation matrix from euler:
    tf::Matrix3x3 R;
    R.setRPY(pose[3],pose[4],pose[5]);
    for (int i=0;i<3;++i)
    {
      t_(i,0)=R[i].x();
      t_(i,1)=R[i].y();
      t_(i,2)=R[i].z();

    }
    c_check.add_obj_mesh(stl_file,t_);
    cerr<<"added object mesh!"<<endl;
    vector<double> col={0.4,0.4,0.9,1.0};

    //vector<double> col={0,1.0,0,1.0};
    // publish object as marker:
    pub_marker(obj_mesh,pose,"obj_marker",col);
    
  }
  */
  void init_robot_model()
  {
    // read parameters from rosparam server:
    
    vector<string> arm_link_names;
    vector<string> hand_link_names;
    string urdf_param;

    int n_links;
    // default params:
    vector<string> ee_names;//={"palm_link","index_tip","middle_tip","ring_tip","thumb_tip"};
    vector<string> base_names;//={"base_link","base_link","base_link","base_link","base_link"};
    vector<string> j_names;
    assert(nh->getParam("urdf_param",urdf_param));
    
    while(!nh->hasParam(urdf_param) && ros::ok())
    {
    }
    
    // read parameters from rosparam server:
    assert(nh->getParam("base_frames",base_names));
    base_frame_=base_names[0];
    assert(nh->getParam("ee_frames",ee_names));
    assert(nh->getParam("joint_names",j_names));
    //assert(nh->getParam("arm_coll_links",arm_link_names));

    assert(nh->getParam("hand_coll_links",hand_link_names));
    assert(nh->getParam("contact_link_index", contact_links));

    // create functions for getting hand state from eigen state
    // store indexing of collision links and contact links:
    // we store collision checking links first and then contact links;
    
    vector<string> robot_link_names=hand_link_names;
    //robot_link_names.insert(robot_link_names.end(),hand_link_names.begin(),hand_link_names.end());

    //int reach_idx;
    //assert(nh->getParam("reachability_frame",reach_idx));
    //cerr<<"****Palm frame name: "<<robot_link_names[reach_idx]<<" "<<reach_idx<<endl;
    //robot_link_names.insert(robot_link_names.end(),hand_contact_link_names.begin(),hand_contact_link_names.end());

    // build collision meshes:
    std::string robot_desc_string;
    
    // read urdf as a string:
    nh->param(urdf_param, robot_desc_string, std::string());
    //c_check.load_robot_links(arm_link_names,hand_link_names,robot_desc_string);

    //cerr<<"Created robot collision objects"<<endl;

    
    // build kdl model:
    vector<double> g_vec={0.0,0.0,-9.8};
    robot_kdl_=manipulator_kdl::robotKDL(urdf_param,*nh,base_names,ee_names,robot_link_names,g_vec);

    
    robot_kdl_.getJointLimits(j_names,robot_kdl_.up_bounds,robot_kdl_.low_bounds);

    
    cerr<<"Created robot kdl object with dof: "<<robot_kdl_.dof<<endl;
    /*
    // read reachability meshes:
    vector<string> reach_meshes;
    assert(nh->getParam("contact_reachability", reach_meshes));
    // append package path:
    std::string mesh_location=ros::package::getPath("ll4ma_robots_description");
    for(int i=0;i<reach_meshes.size();++i)
    {

      reach_meshes[i].insert(0,mesh_location);
    }
    Eigen::Matrix4d T;
    T.setIdentity();
    c_check.add_reach_mesh(reach_meshes,T);
    cerr<<"added contact reachability meshes"<<endl;
    */
  }
  bool in_grasp_call(in_grasp_manipulation::InGraspManipulation::Request &req,in_grasp_manipulation::InGraspManipulation::Response &res)
  {

  Eigen::Map<Eigen::VectorXd> joints_0(&req.init_joints.position[0],req.init_joints.position.size());
  Eigen::VectorXd final_joints;
  if(req.final_joints.position.size()!=0)
  {
    Eigen::Map<Eigen::VectorXd> final_joints(&req.final_joints.position[0],req.final_joints.position.size());
  }
  else
  {
    Eigen::Map<Eigen::VectorXd> final_joints(&req.init_joints.position[0],req.init_joints.position.size());
  }

  vector<Eigen::MatrixXd> data_mat;
  vector<vector<double>> data_arr;

  
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

  

  Eigen::MatrixXd thumb_mat;
  robot_kdl_.getFK(3,joints_0.segment(12,4),thumb_mat);
  
  Eigen::MatrixXd obj_offset=thumb_mat.inverse()*obj_mat;
  
  tf::Pose des_pose_tf;
  tf::poseMsgToTF(req.desired_object_pose,des_pose_tf);
  //cerr<<
  // convert desired object pose to thumb pose:
  R=des_pose_tf.getBasis();
  
  Eigen::MatrixXd des_obj_mat(4,4);
  des_obj_mat.setIdentity();
  for(int i=0;i<3;++i)
  {
    des_obj_mat(i,0)=R[i][0];
    des_obj_mat(i,1)=R[i][1];
    des_obj_mat(i,2)=R[i][2];
  }
  des_obj_mat(0,3)=req.desired_object_pose.position.x;
  des_obj_mat(1,3)=req.desired_object_pose.position.y;
  des_obj_mat(2,3)=req.desired_object_pose.position.z;
  //cerr<<des_obj_mat(0,3)<<endl;
  Eigen::MatrixXd des_thumb_mat=des_obj_mat*obj_offset.inverse();

  // Get des pose from mat:
  //thumb_joints[0]=0.8;
  //vector<double> des_pose_thumb=robot_kdl_.getFK(3,thumb_joints,true);
  vector<double> des_pose;
  des_pose.resize(6);
  des_pose[0]=des_thumb_mat(0,3);
  des_pose[1]=des_thumb_mat(1,3);
  des_pose[2]=des_thumb_mat(2,3);
  // Get euler xyz from rotation matrix:
  tf::Matrix3x3 R_(des_thumb_mat(0,0),des_thumb_mat(0,1),des_thumb_mat(0,2),
                   des_thumb_mat(1,0),des_thumb_mat(1,1),des_thumb_mat(1,2),
                   des_thumb_mat(2,0),des_thumb_mat(2,1),des_thumb_mat(2,2));
  R_.getRPY(des_pose[3],des_pose[4],des_pose[5]);

  
  // Relaxed rigidity data:
  // [des_obj_pose(6x1), rigid_finger,fingers(4x1),position_offset(n_f*3),orientation_offset(n_f*3)]
  
  vector<int> fingers(begin(req.fingers), end(req.fingers));
  vector<double> fingers_d(fingers.begin(),fingers.end());
  vector<double> weights(begin(req.weights),end(req.weights));
  vector<double> r_fin;
  int r_f=req.rigid_finger;
  r_fin.resize(1,r_f);
  data_arr.resize(4+fingers.size()+1); 
  data_arr[0]=des_pose;
  data_arr[1]=weights;
  data_arr[2]=r_fin;
  data_arr[3]=fingers_d;
  //cerr<<fingers_d.size()<<endl;
  Eigen::VectorXd thumb_pose;
  
  robot_kdl_.getFK(r_f,joints_0.segment(12,4),thumb_pose,true);
  vector<double> thumb_pose_arr(thumb_pose.data(),thumb_pose.data()+thumb_pose.size());
  
  data_arr[4+fingers.size()]=thumb_pose_arr;
  // compute offsets in thumb frame:
  // thumb matrix w_T_th=thumb_mat
  //compute inverse:
  Eigen::MatrixXd thumb_inv=thumb_mat.inverse();
  Eigen::VectorXd f_pose;
  Eigen::MatrixXd f_mat;
  vector<double> orient_offset;
  for(int i=0;i<fingers.size();++i)
  {
    int f=int(fingers_d[i]);
    // get joint angles for finger f:
    //get finger mat w_T_f
    robot_kdl_.getFK(f,joints_0.segment(f*4,4),f_mat);

    // Compute offset matrix:
    Eigen::MatrixXd offset_mat=thumb_inv*f_mat;
    data_mat.push_back(offset_mat);
    robot_kdl_.getFK(f,joints_0.segment(f*4,4),f_pose,true);
    
    Eigen::VectorXd pose_diff=thumb_pose-f_pose;
    vector<double> position_offset;
    position_offset.resize(pose_diff.size());
    
    pose_diff[0]=offset_mat(0,3);
    pose_diff[1]=offset_mat(1,3);
    pose_diff[2]=offset_mat(2,3);
    
    // Compute orientation offset:

    vector<double> thumb_orient(thumb_pose_arr.data()+3, thumb_pose_arr.data()+6);
    vector<double> f_orient(f_pose.data()+3, f_pose.data()+6);
    
    robot_kdl_.euler_diff(f_orient,thumb_orient,orient_offset);
    
    //orient_offset.resize(pose_diff.size());
    //VectorXd::Eigen::Map(&orient_offset[0], pose_diff.size()) = pose_diff;
    pose_diff[3]=orient_offset[0];
    pose_diff[4]=orient_offset[1];
    pose_diff[5]=orient_offset[2];
    VectorXd::Map(&position_offset[0], pose_diff.size()) = pose_diff;

    // Computing position offset:
    data_arr[4+i]=position_offset;
    //data_arr[4+i+fingers.size()]=orient_offset;
    
  }

  
  //cerr<<"data size "<<data_arr.size()<<endl;
  vector<double> vel_limits;
  vel_limits.resize(16,req.weights[4]);
  int t_steps_=req.T;
  // Creating opt problem instance:
  cerr<<"creating opt problem"<<endl;
  vector<double> j0(joints_0.data(),joints_0.data()+joints_0.size());
  //opt_problems::inGraspOptProblem opt_problem(t_steps_,robot_kdl_,data_arr,data_mat,vel_limits,j0);
  opt_problems::optProblem opt_problem;
  opt_problem.updateTimesteps(t_steps_);
  opt_problem.updateRobotModel(robot_kdl_);
  opt_problem.updateInitialState(j0);
  opt_problem.updateData(vel_limits,data_arr,data_mat);
  cerr<<"creating pagmo prob"<<endl;
  problem prob{con_optimization{opt_problem}};

  cerr<<"setting tol"<<endl;

  // set tolerances for inequality constraints:
  vector<double> tol(opt_problem.m_nic+opt_problem.m_nec,1e-4);

  // set larger tolerance for rr constraints:
  cerr<<"RR thresholds, position: "<<weights[1]<<" orientation: "<<weights[0]<<endl;
  
  for(int i=0;i<t_steps_;++i)
  {
    //tol[robot_kdl_.dof*t_steps_+i]=weights[1];// wr
    //tol[robot_kdl_.dof*t_steps_+t_steps_+i]=weights[0];// wo

  }
  
  prob.set_c_tol(tol);
  pagmo::snopt7 sn_v(false,"/usr/local/lib/");
  //sn_v.set_numeric_option("Major optimality tolerance",1e-5);
  sn_v.set_integer_option("Scale option",1);

  sn_v.set_integer_option("Verify level",-1);
  sn_v.set_numeric_option("Time limit",5);
    
  sn_v.set_numeric_option("Function precision",1e-14);

  algorithm algo{sn_v};
  algo.set_verbosity(100);
  //algorithm algo{cmaes(500)};
  cerr<<"Creating pop"<<endl;
  population pop{prob,3u};
  
  vector<double> x_in(16*t_steps_,0.0);
  // seed using final joint angles IK:

  for(int i=0;i<t_steps_;++i)
  {    
    for (int j=0;j<16;++j)
    {
      x_in[i*16+j]=joints_0[j];//+(final_joints[j]-joints_0[j])*(i+1)/(t_steps_);
    }
  }
  cerr<<"created pop"<<endl;
  // seed using previous joint trajectory:
  if(req.initial_jtraj.points.size()==t_steps_+1)
  {
    cerr<<"Using old trajectory for initialization"<<endl;
    for(int i=0;i<t_steps_;++i)
    {
      for (int j=0;j<16;++j)
      {
        x_in[i*16+j]=req.initial_jtraj.points[i+1].positions[j];
      }
    }
  }
  if(req.final_joints.position.size()>0)
  {
    cerr<<"Using final js for initialization"<<endl;
    for(int i=1;i<t_steps_;++i)
    {
      for (int j=0;j<16;++j)
      {
        x_in[i*16+j]=req.final_joints.position[j];
      }
    }
  }

  cerr<<"checking initial cost"<<endl;
  vector<double> f_in=prob.fitness(x_in);
  cerr<<"Initial fitness: "<<f_in[0]<<endl;
  if(req.initialize_opt==true)
  {
    pop.set_xf(0,x_in,f_in);
  }

  cerr<<"Solving"<<endl;  
  population pop_res = algo.evolve(pop);
  cerr<<"optimization completed!  Final cost: "<<pop_res.champion_f()[0]<<endl;



  vector<double> j_best=pop_res.champion_x();

  
  trajectory_msgs::JointTrajectory joint_traj;
  std::vector<std::string> joint_names={"index_joint_0","index_joint_1","index_joint_2","index_joint_3",
                                        "middle_joint_0","middle_joint_1","middle_joint_2","middle_joint_3",
                                        "ring_joint_0","ring_joint_1","ring_joint_2","ring_joint_3",
                                        "thumb_joint_0","thumb_joint_1","thumb_joint_2","thumb_joint_3"};
  joint_traj.joint_names=joint_names;    

  vector<double> j_arr;

  j_arr.resize(16);
  trajectory_msgs::JointTrajectoryPoint jt_pt;
  jt_pt.positions=j0;
  joint_traj.points.push_back(jt_pt);
  
  
  for (int i=0;i<j_best.size()/16;i++)
  {
    for(int j=0;j<16;j++)
    {
      j_arr[j]=j0[j];
    }  
 
    for(int k=0; k<fingers.size();++k)
    {
      for (int j=0;j<4;++j)
      {
        j_arr[fingers[k]*4+j]=j_best[i*16+fingers[k]*4+j];
      }
    }
    jt_pt.positions=j_arr;
    joint_traj.points.push_back(jt_pt);
  }

  res.joint_traj=joint_traj;

  // new object pose:
  Eigen::Map<Eigen::VectorXd> th_j(&j_arr[12],4);
  robot_kdl_.getFK(3,th_j,thumb_mat);
  Eigen::MatrixXd  n_object_pose=thumb_mat*obj_offset;
  vector<double> n_pose;
  robot_kdl_.frame_to_pose_(n_object_pose,n_pose,false);
  /*
  // convert object frame to pose:
  tf::Matrix3x3 R_(n_object_pose(0,0), n_object_pose(0,1), n_object_pose(0,2),
                   n_object_pose(1,0), n_object_pose(1,1), n_object_pose(1,2),
                   n_object_pose(2,0), n_object_pose(2,1), n_object_pose(2,2));
  tf::Quaternion q_;
  R_.getRotation(q_);
  */
  geometry_msgs::PoseStamped n_obj_pose_;

  n_obj_pose_.pose.position.x=n_pose[0];
  n_obj_pose_.pose.position.y=n_pose[1];
  n_obj_pose_.pose.position.z=n_pose[2];
  n_obj_pose_.pose.orientation.x=n_pose[3];
  n_obj_pose_.pose.orientation.y=n_pose[4];
  n_obj_pose_.pose.orientation.z=n_pose[5];
  n_obj_pose_.pose.orientation.w=n_pose[6];
  res.n_obj_pose=n_obj_pose_;

  
  // compute object trajectory:
  geometry_msgs::PoseArray obj_pose_traj;
  obj_pose_traj.poses.resize((j_best.size()/16)+1);
  obj_pose_traj.header.frame_id="palm_link";
  // adding initial object pose:

  robot_kdl_.getFK(3,joints_0.segment(12,4),thumb_mat);
  n_object_pose=thumb_mat*obj_offset;
  robot_kdl_.frame_to_pose_(n_object_pose,n_pose,false);
  
  obj_pose_traj.poses[0].position.x=n_pose[0];
  obj_pose_traj.poses[0].position.y=n_pose[1];
  obj_pose_traj.poses[0].position.z=n_pose[2];
  obj_pose_traj.poses[0].orientation.x=n_pose[3];
  obj_pose_traj.poses[0].orientation.y=n_pose[4];
  obj_pose_traj.poses[0].orientation.z=n_pose[5];
  obj_pose_traj.poses[0].orientation.w=n_pose[6];

  Eigen::VectorXd thumb_joints;
  thumb_joints.resize(4);
  for (int i=0;i<j_best.size()/16;i++)
  {
   
    thumb_joints.setZero();
    for (int j=4*r_f;j<4*r_f+4;++j)
    {
      thumb_joints[j-4*r_f]=j_best[i*16+j];

    }

    robot_kdl_.getFK(3,thumb_joints,thumb_mat);
    n_object_pose=thumb_mat*obj_offset;
    robot_kdl_.frame_to_pose_(n_object_pose,n_pose,false);
  
    obj_pose_traj.poses[i+1].position.x=n_pose[0];
    obj_pose_traj.poses[i+1].position.y=n_pose[1];
    obj_pose_traj.poses[i+1].position.z=n_pose[2];
    obj_pose_traj.poses[i+1].orientation.x=n_pose[3];
    obj_pose_traj.poses[i+1].orientation.y=n_pose[4];
    obj_pose_traj.poses[i+1].orientation.z=n_pose[5];
    obj_pose_traj.poses[i+1].orientation.w=n_pose[6];
    
  }

  res.obj_traj=obj_pose_traj;

  // compute metrics:
  // RR cost:
  
  return true;


  }
};

using namespace std;
int main(int argc, char** argv)
{
  //collisionChecker c_check;
  ros::init(argc,argv,"in_grasp_planner_node");
  ros::NodeHandle n;
  srvClass srv_call;
  srv_call.init(n);
  srv_call.init_robot_model();
  //srv_call.init_env();
  //srv_call.init_obj();
  // create ros service:
  //ros::ServiceServer env_service=n.advertiseService( "/grasp_planner/update_world", &srvClass::update_env_cloud, &srv_call);

  ros::ServiceServer service=n.advertiseService("/in_grasp_planner/get_plan",&srvClass::in_grasp_call,&srv_call);
  ROS_INFO("In-grasp planner service ready");
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
   
    loop_rate.sleep();
    ros::spinOnce();
  }
  //return 0;
}
                                               
 

