#include <finger_relocation/problem/rrt_finger_gait.h>
#include <in_hand_manipulation_msgs/RelocateFingerKin.h>
#include <in_hand_manipulation_msgs/UpdateObj.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_broadcaster.h>

ros::Publisher pub;
class fgPlanner
{
public:
  manipulator_kdl::robotKDL allegro_kdl;
  handEnv c_check;
  ros::NodeHandle* nh;
  ros::Publisher m_pub;
  void init(ros::NodeHandle &n)
  {
    nh=&n;
    // initialize collision checker
    c_check=handEnv(0.002);
    
    vector<double> g_vec={0.0,0.0,-9.8};
        
    vector<string> hand_link_names={"index_link_1","index_link_2","index_tip", "middle_link_1","middle_link_2","middle_tip","ring_link_1","ring_link_2","ring_tip", "thumb_link_2","thumb_link_2","thumb_tip"};

    // initialize collision environment:
    std::string robot_desc_string;
    
    // read urdf as a string:
    nh->param("robot_description", robot_desc_string, std::string());
    c_check.load_hand_links(hand_link_names,robot_desc_string);


    vector<string> hand_base_names={"palm_link","palm_link","palm_link","palm_link"};
    vector<string> hand_ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"}; 
    allegro_kdl=manipulator_kdl::robotKDL("robot_description",*nh,hand_base_names,hand_ee_names,hand_link_names,g_vec);
    allegro_kdl.urdfParser();
    allegro_kdl.dof=16;
      
    cerr<<"Created robot collision files"<<endl;
    
  }

  bool update_obj(in_hand_manipulation_msgs::UpdateObj::Request &req, in_hand_manipulation_msgs::UpdateObj::Response &res)
  {
    string mesh=req.mesh_file;
    vector<double> pose(7,0.0);
    pose[0]=req.obj_pose.position.x;
    pose[1]=req.obj_pose.position.y;
    pose[2]=req.obj_pose.position.z;
    pose[3]=req.obj_pose.orientation.x;
    pose[4]=req.obj_pose.orientation.y;
    pose[5]=req.obj_pose.orientation.z;
    pose[6]=req.obj_pose.orientation.w;
    
    
    // load obj mesh
    c_check.load_obj(mesh);

    // update object pose
    c_check.update_grasp_obj_pose(pose);

    
    // visualize object pointcloud:
    m_pub=nh->advertise<visualization_msgs::MarkerArray>("collision_env", 1);
    static tf::TransformBroadcaster br;
    ros::Rate l(100);

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose[0],pose[1],pose[2]) );
    tf::Quaternion q(pose[3],pose[4],pose[5],pose[6]);
    transform.setRotation(q);

    visualization_msgs::MarkerArray oc_map; 
    c_check.get_env_markers(oc_map,"obj_frame");

    // visualize pointcloud:
    for(int i=0;i<100;++i)
    {
       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "palm_link", "obj_frame"));

      // visualize octomap:
      m_pub.publish(oc_map);
      l.sleep();
    }

    // pcl::PointCloud<PointXYZRGB>::Ptr cloud_obj1(new PointCloud<PointXYZRGB>);
    //c_check.get_obj_pointcloud(cloud_obj1);
    //pub.publish(cloud_obj1);
    return true;
  }
  bool fg_call(in_hand_manipulation_msgs::RelocateFingerKin::Request &req, in_hand_manipulation_msgs::RelocateFingerKin::Response &res)
  { // Getting parameters from service request:
    int f=req.f_idx;

    cerr<<"Calling fg service"<<endl;
    vector<double> joints_0=req.js.position;
    Eigen::VectorXd des_pt;//(6);
    des_pt.resize(6);
    des_pt[0]=req.des_contact_pt.x;
    des_pt[1]=req.des_contact_pt.y;
    des_pt[2]=req.des_contact_pt.z;

    // contact normals:
    des_pt[3]=req.des_contact_normal.x;
    des_pt[4]=req.des_contact_normal.y;
    des_pt[5]=req.des_contact_normal.z;
    int t_steps=req.T;
    vector<double> vel_limits;//(16,0.2);

    if(t_steps<3)
    {
      vel_limits.resize(16,100.2);
    }
    else
    {
      vel_limits.resize(16,0.2);

    }
   
    cerr<<"reading joint angles"<<endl;
    // setup collision checking
    Eigen::VectorXd hand_q(16);
    // initialize object meshes to initial joint state:
    for(int j=0; j<allegro_kdl.dof;++j)
    {
      hand_q[j]=joints_0[j];
    }

    vector<vector<double>> link_poses;
    link_poses.resize(3*4);
  
    // get link pose for every link:
    Eigen::VectorXd finger_q;
    finger_q.resize(4);
    Eigen::VectorXd l_pose_;
    cerr<<"computing link poses"<<endl;
    for (int j=0;j<4;j++)
    {
      //finger joints:
      finger_q.tail(4)=hand_q.segment(4*j,4);
      for(int i=0;i<3;++i)
      {
        allegro_kdl.getLinkPose(j*3+i,finger_q,l_pose_);
        allegro_kdl.getSTDVector(l_pose_,link_poses[j*3+i]);
      }
    }
    cerr<<"updating hand state"<<endl;
    // get contact data:
    c_check.update_hand_state(link_poses);
    cerr<<"updated hand state"<<endl;
    // update link collision index:
    vector<int> f_idx;
    for(int i=0;i<4;++i)
    {
      if(i!=f)
      {
        for(int j=0;j<3;++j)
        {
          f_idx.push_back(i*3+j);
        }
      }
    }

    c_check.update_collision_links(f_idx);

    

    vector<double> joint_init=joints_0;

    Eigen::VectorXd f_j0(4);
    for(int i=0;i<4;i++)
    {
      f_j0[i]=joint_init[f*4+i];
    }
    Eigen::VectorXd des_pt_=des_pt.head(3);
    // initialize problem:

    ompl_wrapper::fgProblem fg_problem;
    allegro_kdl.dof=4;
    // change bounds:
    vector<double> up_bounds(4);
    vector<double> low_bounds(4);
    for(int i=0;i<4;i++)
    {
      up_bounds[i]=allegro_kdl.up_bounds[f*4+i];
      low_bounds[i]=allegro_kdl.low_bounds[f*4+i];
    }
    allegro_kdl.up_bounds=up_bounds;
    allegro_kdl.low_bounds=low_bounds;
    fg_problem.init_planner(allegro_kdl);
    fg_problem.setupGoal(des_pt_,f,c_check);



    
    
    //return -1;
    cerr<<"Solving"<<endl;  
    bool solution=fg_problem.solve(4.0,f_j0);



    trajectory_msgs::JointTrajectory joint_traj;
    std::vector<std::string> joint_names={"index_joint_0","index_joint_1","index_joint_2","index_joint_3",
                                        "middle_joint_0","middle_joint_1","middle_joint_2","middle_joint_3",
                                          "ring_joint_0","ring_joint_1","ring_joint_2","ring_joint_3",
                                          "thumb_joint_0","thumb_joint_1","thumb_joint_2","thumb_joint_3"};
    joint_traj.joint_names=joint_names;    
  
    vector<double> j_arr;
    
    j_arr.resize(16);
    trajectory_msgs::JointTrajectoryPoint jt_pt;
    jt_pt.positions=joints_0;
    joint_traj.points.push_back(jt_pt);
    vector<int> j_idx;
    vector<Eigen::VectorXd> j_best=fg_problem.q_solution;
    for (int i=0;i<j_best.size();i++)
    {
      for(int k=0;k<4;++k)
      {
        if(k==f)
        {
          for(int j=0;j<4;j++)
          {
            j_arr[k*4+j]=j_best[i][j];
          }
        }
        else
        {
          for(int j=0;j<4;j++)
          {
            j_arr[k*4+j]=joints_0[k*4+j];
          }
        }
      }
      jt_pt.positions=j_arr;
      joint_traj.points.push_back(jt_pt);
    }
    
    res.joint_traj=joint_traj;

    // compute next contact point from fk:
    geometry_msgs::PointStamped cpt;
    Map<Eigen::VectorXd> final_joints(&joint_traj.points[joint_traj.points.size()-1].positions[f*4],4);
    cerr<<final_joints.transpose()<<endl;
    Eigen::VectorXd f_p;
    allegro_kdl.getFK(f,final_joints,f_p);
    // get contact point:
    //cerr<<f_p<<endl;
    /*
    vector<double> ftip_pose;
    allegro_kdl.getSTDVector(f_p,ftip_pose);
    coll_data ftip_data;
    c_check.check_ftip_contact(ftip_pose,ftip_data,f*3+2);
    cerr<<ftip_data.collision<<" "<<f*3+2<<endl;
    cpt.point.x=ftip_data.p1.x();
    cpt.point.y=ftip_data.p1.y();
    cpt.point.z=ftip_data.p1.z();
    */
    cpt.point.x=f_p[0];
    cpt.point.y=f_p[1];
    cpt.point.z=f_p[2];
    res.next_contact_pt=cpt;
    //cerr<<final_joints<<endl;
    return true;
  }
  
  
};


int main(int argc,char** argv)
{
  ros::init(argc,argv,"fg_rrt_server");
  ros::NodeHandle n;
  fgPlanner fg_planner;
  fg_planner.init(n);

  ros::ServiceServer service=n.advertiseService("/in_hand/rrt/finger_gait",&fgPlanner::fg_call,&fg_planner);
  ros::ServiceServer obj_service=n.advertiseService("/in_hand/rrt/finger_gait/update_obj",&fgPlanner::update_obj,&fg_planner);
  ros::spin();// Loop rate?
}
