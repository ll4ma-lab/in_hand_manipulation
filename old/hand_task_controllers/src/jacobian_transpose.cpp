// Takes end-effector position in the palm frame as input and sends joint position command using Jacobian transpose control
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/package.h>


#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

using namespace std;











float x=0,y=0,z=0;
double q[4];
double old_pt[3];
double curr_pt[3];
double des_pt[3];
double err[3];
bool control_init=true;
bool got_position=false;
KDL::Chain index_chain;







// Subscribe to desired end-effector position
void pointCallback(const geometry_msgs::PointStamped pt)
{
  des_pt[0]=pt.point.x;
  des_pt[1]=pt.point.y;
  des_pt[2]=pt.point.z;
  got_position=true;
}

// Subscribe to  current joint angles
void stCallback(const sensor_msgs::JointState j)
{
  if(got_position==true)
    {
  float K=1.0;
  float D=0.1;
  KDL::JntArray joint_angles;
  joint_angles.resize(4);
  for(int i=0;i<4;i++)
    {
      joint_angles.data[i]=j.position[i];

    }
  KDL::ChainFkSolverPos_recursive fksolver(index_chain);
  KDL::ChainJntToJacSolver index_jacobian_solver(index_chain);
  KDL::Frame cartpos;
  // Compute current end-effector position
  fksolver.JntToCart(joint_angles,cartpos);
  for(int i=0;i<3;i++)
    {
      curr_pt[i]=cartpos.p.data[i];
      
      if(control_init==true)
	{
	  old_pt[i]=curr_pt[i];
	}
    }
  control_init=false;
  KDL::Jacobian J;// is 6x4
  J.resize(index_chain.getNrOfJoints());
  index_jacobian_solver.JntToJac(joint_angles,J);
  float err=0;
  for(int i=0;i<3;i++)
    {
      err=K*(des_pt[i]-curr_pt[i])+D*(curr_pt[i]-old_pt[i]);

      for(int k=0;k<4;k++)
	{
	  q[k]+=J(i,k)*err;

	}
    }

  copy(std::begin(curr_pt), std::end(curr_pt), std::begin(old_pt));

    }
}


int main(int argc,char** argv)
{
  des_pt[0]=0.0;
  des_pt[1]=0.0;
  des_pt[2]=0.0;
  ros::init(argc,argv,"allegro_op_space");
  ros::NodeHandle n;
  ros::Rate r(100);

  ros::Subscriber cmd_sub=n.subscribe<geometry_msgs::PointStamped>("/allegro_hand_right/point_cmd",1000,pointCallback);
  ros::Subscriber joint_sub=n.subscribe<sensor_msgs::JointState>("/allegro_hand_right/joint_states",1000,stCallback);
  
  ros::Publisher joint_pub=n.advertise<sensor_msgs::JointState>("/allegro_hand_right/joint_cmd",1);


  sensor_msgs::JointState joint_cmd;
  std::string names[]={"index_joint_0","index_joint_1","index_joint_2", "index_joint_3",
                   "middle_joint_0","middle_joint_1","middle_joint_2", "middle_joint_3",
                   "ring_joint_0","ring_joint_1","ring_joint_2", "ring_joint_3",
                   "thumb_joint_0","thumb_joint_1","thumb_joint_2", "thumb_joint_3"};
 
  std::vector<std::string> name_vect(names,names+sizeof(names)/sizeof(std::string)); //make it const if you can
  joint_cmd.name=name_vect;
  joint_cmd.position.resize(16);


  KDL::Tree tree;
  string path = ros::package::getPath("urlg_robots_description");
  //  path.append("/urdf/allegro_right/allegro_hand_description_right.urdf");
  path.append("/robots/allegro_right.urdf");

  if (!kdl_parser::treeFromFile(path, tree)) 
    {
        printf(" Failed to construct kdl tree\n");
        return -1;
    }
  string base_link="palm_link";
  vector<string> ee_names={"index_tip", "middle_tip", "ring_tip", "thumb_tip"};
  string e_name=ee_names[0];
  tree.getChain(base_link,e_name,index_chain);


  while(n.ok())
    {

      if(got_position==true)
      // Publish joint angles
	{
      joint_cmd.position[0]=q[0];
      joint_cmd.position[1]=q[1];
      joint_cmd.position[2]=q[2];
      joint_cmd.position[3]=q[3];
      joint_cmd.header.stamp=ros::Time::now();
      joint_pub.publish(joint_cmd);
	}
      ros::spinOnce();
      r.sleep();
    }
  return 0;
}
