#include "finger_relocation/problem/hand_problem.hpp"
#include <ll4ma_opt_wrapper/pagmo_opt/wrapper.hpp>
#include <in_hand_manipulation_msgs/RelocateFingerKin.h>
#include <in_hand_manipulation_msgs/UpdateObj.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_broadcaster.h>

using namespace pagmo;
ros::Publisher pub;
class fgPlanner
{
public:
  //manipulator_kdl::robotKDL allegro_kdl;
  //handEnv c_check;
  in_hand_manipulation::handEnv hand_env;
  bool collision_ready = false;
  ros::NodeHandle* nh;
  ros::Publisher m_pub;
  void init(ros::NodeHandle &n)
  {
    nh=&n;
    cerr<<"INitializing..."<<endl;
    in_hand_manipulation::handEnv rob_env_checker(n);
    hand_env = rob_env_checker;
    hand_env.load_robot_model();
    cerr<<"Initialized robot"<<endl;
    hand_env.load_obj();
    collision_ready = true;
    // initialize collision checker
      
    cerr<<"Created robot environment model"<<endl;
    
  }
  bool add_obj_call(ll4ma_opt_utils::AddObject::Request &req, ll4ma_opt_utils::AddObject::Response &res)
  {
    return hand_env.load_obj_call(req,res);
  }
  bool fg_call(in_hand_manipulation_msgs::RelocateFingerKin::Request &req, in_hand_manipulation_msgs::RelocateFingerKin::Response &res)
  {

    int finger;
    // desired pose for finger
    
    /*
    // Getting parameters from service request:
    int f=req.f_idx;

    cerr<<"Calling fg service"<<endl;
    vector<double> joints_0=req.js.position;
    Eigen::VectorXd des_pt;//(6);
    if(req.des_js.position.size()>0)
    {
      des_pt.resize(4);
      des_pt[0]=req.des_js.position[f*4];
      des_pt[1]=req.des_js.position[f*4+1];
      des_pt[2]=req.des_js.position[f*4+2];
      des_pt[3]=req.des_js.position[f*4+3];

    }
    else
    {
      des_pt.resize(6);
      des_pt[0]=req.des_contact_pt.x;
      des_pt[1]=req.des_contact_pt.y;
      des_pt[2]=req.des_contact_pt.z;

      // contact normals:
      des_pt[3]=req.des_contact_normal.x;
      des_pt[4]=req.des_contact_normal.y;
      des_pt[5]=req.des_contact_normal.z;
    }
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

    // get desired point for allegro :
    //Eigen::VectorXd jpt;
    //allegro_kdl.getFK(f,hand_q.segment(f*4,4),jpt);
    cerr<<"f: "<<f<<endl;
    //cerr<<hand_q.transpose()<<endl;
    //cerr<<jpt.transpose()<<endl;
    // get desired point from service call:
    
    opt_problems::optProblem opt_problem(t_steps,allegro_kdl,des_pt,vel_limits,joints_0,f,c_check);
    
    problem prob{trajectory_optimization{opt_problem}};
    vector<double> tol(opt_problem.m_nic,1e-4);
    
    prob.set_c_tol(tol);
    pagmo::snopt7 sn_v(false,"/usr/local/lib/");
    //sn_v.set_integer_option("Iteration limit",200);
    //sn_v.set_integer_option("Major Iteration limit",100);
    //sn_v.set_integer_option("Iteration limit",100);

    //sn_v.set_numeric_option("Major optimality tolerance",1e-3); 
    //sn_v.set_numeric_option("Linesearch tolerance",0.3); 
    
    algorithm algo{sn_v};
    algo.set_verbosity(1);
    //algorithm algo{cmaes(500)};
    population pop{prob,1u};
    
    vector<double> x_in(16*t_steps,0.0);
    vector<double> joint_init=joints_0;
    //joint_init[f*4+1]=0.1;
    //joint_init[f*4+2]=0.2;
    //joint_init[f*4+3]=0.1;
        
    for(int i=0;i<t_steps;++i)
    {    
      for (int j=0;j<16;++j)
      {
        x_in[i*16+j]=joint_init[j];//+(final_joints[j]-joints_0[j])*(i+1)/(t_steps_);
      }
    }
    if(req.init_jtraj.points.size()==t_steps+1)
    {
      for(int i=1;i<req.init_jtraj.points.size();++i)
      {
        for(int j=0;j<req.init_jtraj.points[i].positions.size();++j)
        {
          x_in[(i-1)*16+j]=req.init_jtraj.points[i].positions[j];
        }
      }
              
    }
    else if(req.init_jtraj.points.size()>1 && req.init_jtraj.points.size()<t_steps)
    {
      cerr<<"Initializing from traj"<<endl;
      for(int i=1;i<t_steps;++i)
      {
        for(int j=0;j<req.init_jtraj.points[1].positions.size();++j)
        {
          x_in[(i)*16+j]=joint_init[j]+(req.init_jtraj.points[1].positions[j]-joint_init[j])*double(i)/t_steps;
          //x_in[(i)*16+j]=req.init_jtraj.points[1].positions[j];
        }
      }
              
    }

    vector<double> f_in=prob.fitness(x_in);
    
    //vector<double> f_in_1=prob.fitness(x_in);
    cerr<<"Initial fitness: "<<f_in[0]<<endl;
  
    pop.set_xf(0,x_in,f_in);
    //return -1;
    cerr<<"Solving"<<endl;  
    population pop_res = algo.evolve(pop);
    cerr<<"optimization completed!  Final cost: "<<pop_res.champion_f()[0]<<" "<<pop_res.champion_f()[1+allegro_kdl.dof*1+1]<<endl;
    
    // check values:
    vector<double> f_out=prob.fitness(pop_res.champion_x());
    cerr<<"Test Final cost: "<<f_out[0]<<" "<<f_out[1+allegro_kdl.dof*1+1]<<" "<<f_out[1+allegro_kdl.dof*1+1+1+1]<<endl;


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
    cpt.point.x=f_p[0];
    cpt.point.y=f_p[1];
    cpt.point.z=f_p[2];
    res.next_contact_pt=cpt;
    //cerr<<final_joints<<endl;
    */
    return true;
  }
  
  
};


int main(int argc,char** argv)
{
  ros::init(argc,argv,"fg_server");
  ros::NodeHandle n;
  fgPlanner fg_planner;
  fg_planner.init(n);

  ros::ServiceServer service=n.advertiseService("/in_hand/finger_gait",&fgPlanner::fg_call,&fg_planner);
    ros::ServiceServer service_obj=n.advertiseService("/in_hand/finger_gait/update_obj", &fgPlanner::add_obj_call, &fg_planner);

  ros::spin();// Loop rate?
}
