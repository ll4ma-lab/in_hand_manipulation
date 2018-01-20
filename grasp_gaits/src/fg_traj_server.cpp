#ifndef SPARSE_GRADIENT
#define SPARSE_GRADIENT true
#endif
#include "grasp_gaits/problem/finger_gait.h"
#include <in_hand_manipulation/pagmo_opt/trajectory_optimization.hpp>
#include <grasp_gaits/RelocateFingerKin.h>
#include <grasp_gaits/UpdateObj.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <ros/package.h>
using namespace pagmo;
ros::Publisher pub;
class fgPlanner
{
public:
  manipulator_kdl::robotKDL allegro_kdl;
  allegroEnv c_check;
  ros::NodeHandle* nh;
  void init(ros::NodeHandle &n)
  {
    nh=&n;
    // initialize collision checker
    c_check=allegroEnv(0.01,0.01);
    // setup optimization:
    string urdf_file="/home/bala/catkin_ws/src/in_hand_manipulation/in_grasp_manipulation/urdf/allegro.urdf";
    
    // Load allegro kdl object:
    vector<string> ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
    vector<string> base_names={"palm_link","palm_link","palm_link","palm_link"};
    vector<string> link_names={"index_link_1","index_link_2","index_tip", "middle_link_1","middle_link_2","middle_tip", "ring_link_1","ring_link_2","ring_tip", "thumb_link_2","thumb_link_3","thumb_tip"};

    vector<double> g_vec={0.0,0.0,0.0};// Gravity needed for dynamics

    allegro_kdl=manipulator_kdl::robotKDL(urdf_file,base_names,ee_names,link_names,g_vec);
    allegro_kdl.urdfParser();
    
    
    // load robot collision checking env:
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
    cerr<<"Created robot collision files"<<endl;
    
  }

  bool fg_call(grasp_gaits::RelocateFingerKin::Request &req, grasp_gaits::RelocateFingerKin::Response &res)
  { // Getting parameters from service request:
    
    vector<double> joints_0=req.js.position;
    Eigen::VectorXd des_pt(3);
    des_pt[0]=req.des_contact_pt.x;
    des_pt[1]=req.des_contact_pt.y;
    des_pt[2]=req.des_contact_pt.z;
    int t_steps=req.T;
    vector<double> vel_limits(16,0.2);

    if(t_steps<3)
    {
      vector<double> vel_limits(16,100.2);

    }
   
    int f=req.f_idx;
    
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
    
    // get contact data:
    c_check.update_hand_state(link_poses);
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
    opt_problems::optProblem opt_problem(t_steps,allegro_kdl,des_pt,vel_limits,joints_0,f,c_check);
    
    problem prob{trajectory_optimization{opt_problem}};
    vector<double> tol(opt_problem.m_nic,1e-3);
    
    prob.set_c_tol(tol);
    pagmo::snopt7 sn_v(false,"/usr/local/lib/");
    sn_v.set_integer_option("Iteration limit",500);
    sn_v.set_numeric_option("Major optimality tolerance",1e-3); 

    algorithm algo{sn_v};
    algo.set_verbosity(100);
    //algorithm algo{cmaes(500)};
    population pop{prob,10u};
    
    vector<double> x_in(16*t_steps,0.0);
    vector<double> joint_init=joints_0;
    joint_init[f*4+2]=0.1;
    joint_init[f*4+3]=0.1;
        
    for(int i=0;i<t_steps;++i)
    {    
      for (int j=0;j<16;++j)
      {
        x_in[i*16+j]=joint_init[j];//+(final_joints[j]-joints_0[j])*(i+1)/(t_steps_);
      }
    }
    vector<double> f_in=prob.fitness(x_in);
    
    //vector<double> f_in_1=prob.fitness(x_in);
    cerr<<"Initial fitness: "<<f_in[0]<<endl;
  
    pop.set_xf(0,x_in,f_in);
    
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
    jt_pt.positions=joints_0;
    joint_traj.points.push_back(jt_pt);
    vector<int> j_idx;
    
    for (int i=0;i<j_best.size()/16;i++)
    {
      for(int k=0;k<4;++k)
      {
        if(k==f)
        {
          for(int j=0;j<4;j++)
          {
            j_arr[k*4+j]=j_best[i*16+k*4+j];
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
    Eigen::VectorXd f_p;
    allegro_kdl.getFK(f,final_joints,f_p);
    cpt.point.x=f_p[0];
    cpt.point.y=f_p[1];
    cpt.point.z=f_p[2];
    res.next_contact_pt=cpt;
    
    return true;
  }
  
  
};


int main(int argc,char** argv)
{
  ros::init(argc,argv,"fg_traj_server");
  ros::NodeHandle n;
  fgPlanner fg_planner;
  fg_planner.init(n);

  ros::ServiceServer service=n.advertiseService("/in_hand/finger_gait_traj",&fgPlanner::fg_call,&fg_planner);

  ros::spin();// Loop rate?
}
