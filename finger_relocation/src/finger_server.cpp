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

    int finger = req.f_idx;
    // timesteps:
    int t_steps = req.T;
    // desired contact point:
    
    // TODO: setup planning arm+hand

    hand_env.hand_dof_=16;
    
    // desired contact point+contact normal for finger:
    Eigen::VectorXd des_pt;
    des_pt.resize(6);
    des_pt[0]=req.des_contact_pt.x;
    des_pt[1]=req.des_contact_pt.y;
    des_pt[2]=req.des_contact_pt.z;
    
    // contact normals:
    
    des_pt[3]=req.des_contact_normal.x;
    des_pt[4]=req.des_contact_normal.y;
    des_pt[5]=req.des_contact_normal.z;

    // initial joint state:
    // split into arm+hand:
    // TODO: Make this generic!
    vector<double> arm_js,hand_js;
    for(int i=0;i<7; ++i)
    {
      arm_js.emplace_back(req.js.position[i]);
    }
    for(int i=0;i<16; ++i)
    {
      hand_js.emplace_back(req.js.position[7+i]);
    }

    hand_env.update_arm_state(arm_js);

    cout<<"Initializing opt problem"<<endl;
    opt_problems::optProblem opt_problem;
    cout<<"Updating scene"<<endl;


    opt_problem.updateTimesteps(t_steps);

    opt_problem.updateScene(hand_env);
    cout<<"Updating tstep"<<endl;

    cout<<"Updating state"<<endl;

    opt_problem.updateInitialState(hand_js);

    
    // update data for problem:
    
    vector<double> vel_limits;//(16,0.2);

    if(t_steps<3)
    {
      vel_limits.resize(16,100.2);
    }
    else
    {
      vel_limits.resize(16,0.2);
    }
    cout<<"Updating opt problem data"<<endl;
        
    opt_problem.updateData(des_pt,vel_limits,finger);
    cout<<"Updated opt problem data"<<endl;
    //return false;

    cerr<<"initializing seed.."<< opt_problem.bounds()[0].size()<<endl;

    vector<double> x_in(hand_env.hand_dof_ * t_steps,0.5);
    // get seed:
    for(int i=0;i<t_steps;++i)
    {    
      for (int j=0;j<hand_env.hand_dof_;++j)
      {
        x_in[j]=hand_js[j];
        //cerr<<x_in[j]<<endl;
      }
    }
    // cout<<"initializing seed.."<< opt_problem.bounds()[0].size()<<endl;

    //cout<<"Prob initialized"<<opt_problem.bounds()[0].size()<<std::endl;
    problem prob{con_optimization{opt_problem}};

    vector<double> tol(opt_problem.m_nic,1e-4);
    //tol[opt_problem.m_nic-1]=1e-2;
    prob.set_c_tol(tol);

    pagmo::snopt7 sn_v(false,"/usr/local/lib/");
    
    sn_v.set_integer_option("Verify level",-1);
    
    sn_v.set_integer_option("Scale option",0);
    //sn_v.set_integer_option("Violation limit",100);
    sn_v.set_numeric_option("Time limit",30);

    
    sn_v.set_numeric_option("Function precision",1e-10);
    algorithm algo{sn_v};
    algo.set_verbosity(100);
    cout<<"Creating population"<<std::endl; 
    unsigned int init=1u;
    population pop{prob,init};
    cout<<"computing initial fitness"<<endl;
    vector<double> f_in=prob.fitness(x_in);
    pop.set_xf(0,x_in,f_in);

    cout<<"Runnnig snopt"<<std::endl;
    population pop_res=algo.evolve(pop);
    cout<<"Final cost:"<<pop_res.champion_f()[0]<<endl;

    // get joint trajectory
    std::vector<double> x_best;
    std::vector<double> final_joint;
    
    x_best=pop_res.champion_x();

    // joint trajectory?
    // Arm also?
    trajectory_msgs::JointTrajectory joint_traj;
    std::vector<std::string> joint_names=req.js.name;
    joint_traj.joint_names=joint_names;
    
    std::vector<double> joint_angles;
    joint_angles.resize(hand_env.robot_kdl_.dof,0.0);

    trajectory_msgs::JointTrajectoryPoint jt_pt0;
    jt_pt0.positions=req.js.position;
    joint_traj.points.push_back(jt_pt0);

    int arm_dof = 7;
    int hand_dof = 16;
    int T=20;
    for (int i=0;i<t_steps;i++)
    { 
      trajectory_msgs::JointTrajectoryPoint jt_pt;
      for(int j=0;j<arm_dof;j++)
      {
        joint_angles[j]=jt_pt0.positions[j];
      }
      // add preshape:
      for(int j=0;j<hand_dof;j++)
      {
        joint_angles[j+arm_dof]=x_best[i*hand_dof+j];
        //cerr<<i*hand_dof+j<<" "<<x_best[i*hand_dof+j]<<endl;
      }
      jt_pt.positions=joint_angles;
      joint_traj.points.push_back(jt_pt);
      jt_pt.time_from_start=ros::Duration(float(i)*T/float(t_steps));

    }

    res.joint_traj = joint_traj;
    return true;
    /*
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
  }
  
  
};


int main(int argc,char** argv)
{
  ros::init(argc,argv,"fg_server");
  ros::NodeHandle n;
  fgPlanner fg_planner;
  fg_planner.init(n);

  ros::ServiceServer service=n.advertiseService("/in_hand/finger_gait/plan",&fgPlanner::fg_call,&fg_planner);
    ros::ServiceServer service_obj=n.advertiseService("/in_hand/finger_gait/update_obj", &fgPlanner::add_obj_call, &fg_planner);

  ros::spin();// Loop rate?
}
