#include "allegro_op_space_control/jacobian_controller.h"

using namespace jacobian_controller_ns;



// Subscriber callbacks
void PositionController::pointCallback(geometry_msgs::PointStamped pt)
{

  xd_[0]=pt.point.x;
  xd_[1]=pt.point.y;
  xd_[2]=pt.point.z;
  got_position_=true;

}

void PositionController::stCallback(sensor_msgs::JointState j)
{
  if(got_position_==true && initialized_controller_==true)
    {

      for(int i=0;i<4;i++)
	{
	  q_.data[i]=j.position[i];
	}

      index_pose_solver_->JntToCart(q_,cart_pos_);
      for(int i=0;i<3;i++)
	{
	  x_[i]=cart_pos_.p.data[i];
	  if(control_init_==true)
	    {
	      x_old_[i]=x_[i];
	    }
	}
      control_init_=true;

      index_jac_solver_->JntToJac(q_,J_);
      for(int i=0;i<3;i++)
	{
	  x_err[i]=Kp_[i]*(xd_[i]-x_[i])+Kd_[i]*(x_[i]-x_old_[i]);
	  for(int k=0;k<4;k++)
	    {
	      q_control_[k]+=J_(i,k)*x_err[i];
	    }
	  x_old_[i]=x_[i];// Copying current position to previous position
	}
      std::cerr<<x_err[0]<<std::endl;
      // Publish new command
      for( int i=0;i<4;i++)
	{
	  joint_cmd_.position[i]=q_control_[i];
	  q_control_[i]=0.0;
	}

      control_pub_.publish(control_type_);
      joint_pub_.publish(joint_cmd_);
      
    }
}

// Controller initialization in non-realtime
int PositionController::init(ros::NodeHandle &n)
{

  // Loading robot model from URDF file TODO: Import from parameter server instead
  std::string path = ros::package::getPath("urlg_robots_description");
  path.append("/robots/allegro_right.urdf");
  
  if (!kdl_parser::treeFromFile(path, allegro_tree_)) 
    {
        printf(" Failed to construct kdl tree\n");
	// return -1;
    }
  std::string base_link="palm_link";
  std::vector<std::string> ee_names={"index_tip", "middle_tip", "ring_tip", "thumb_tip"};
  std::string e_name=ee_names[0];

  // Initialize KDL stuff 

  
  
  allegro_tree_.getChain(base_link,e_name,index_chain_);
  
  index_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(index_chain_));
  index_jac_solver_.reset(new KDL::ChainJntToJacSolver(index_chain_));
  q_.resize(index_chain_.getNrOfJoints());
  J_.resize(index_chain_.getNrOfJoints());

  // Initialize subscribers and publishers
  joint_pub_=n.advertise<sensor_msgs::JointState>("/allegro_hand_right/joint_cmd",1);
  control_pub_=n.advertise<std_msgs::Int8>("/allegro_hand_right/control_type",1);


  // initialize joint cmd names
  
  std::string names[]={"index_joint_0","index_joint_1","index_joint_2", "index_joint_3",
                   "middle_joint_0","middle_joint_1","middle_joint_2", "middle_joint_3",
                   "ring_joint_0","ring_joint_1","ring_joint_2", "ring_joint_3",
                   "thumb_joint_0","thumb_joint_1","thumb_joint_2", "thumb_joint_3"};
 
  std::vector<std::string> name_vect(names,names+sizeof(names)/sizeof(std::string)); //make it const if you can
  joint_cmd_.name=name_vect;
  joint_cmd_.position.resize(16);

  control_type_.data=0;  
  // Gains
  for(int i=0;i<3;i++)
    {
      Kp_[i]=1.0;
      Kd_[i]=0.1;
    }
  initialized_controller_=true;
  ROS_INFO("intialized controller");
}




// Subscriber callbacks
void ForceController::pointCallback(geometry_msgs::PoseArray pt_ar)
{
  int k=0;
  for (int i=0;i<4;i++)
    {
      k=i*3;

      xd_[k]=pt_ar.poses[i].position.x;
      xd_[k+1]=pt_ar.poses[i].position.y;
      xd_[k+2]=pt_ar.poses[i].position.z;

    }
  got_desired_=true;

}

void ForceController::stCallback(sensor_msgs::JointState j)
{
  if(got_desired_==true&& initialized_controller_==true)
    {
      for (int f=0;f<4;f++)
	{
	  // Since we cannot sense torques from Allegro motors, force control is implemented in open loop

	  for(int i=0;i<4;i++)
	    {
	      q_[f].data[i]=j.position[i];
	    }

	  jac_solver_[f]->JntToJac(q_[f],J_[f]);
	  


	  for(int i=0;i<3;i++)
	    {
	      for(int k=0;k<4;k++)
		{

		  tau_control_[k+f*4]+=J_[f](i,k)*xd_[f*3+i];
		}
	    }
	  k_jac=J_[f].data.transpose()*k_gains*J_[f].data;
	 
	  for(int i=0;i<4;i++)
	    {
	      tau_control_[f*4+i]=k_jac[i]*tau_control_[f*4+i];
	    }
	  //
	}
      // Publish new command
      for( int i=0;i<16;i++)
	{
	  joint_cmd_.effort[i]=tau_control_[i];
	  tau_control_[i]=0.0;
	}

      control_pub_.publish(control_type_);

      joint_pub_.publish(joint_cmd_);
    }
}

// Controller initialization in non-realtime
int ForceController::init(ros::NodeHandle &n)
{

  // Loading robot model from URDF file TODO: Import from parameter server instead
  std::string path = ros::package::getPath("urlg_robots_description");
  path.append("/robots/allegro_right.urdf");
  
  if (!kdl_parser::treeFromFile(path, allegro_tree_)) 
    {
        printf(" Failed to construct kdl tree\n");
	// return -1;
    }
  std::string base_link="palm_link";
  std::vector<std::string> ee_names={"index_tip", "middle_tip", "ring_tip", "thumb_tip"};


  // Initialize KDL stuff 
  for (int i=0;i<4;i++)
    {    
      allegro_tree_.getChain(base_link,ee_names[i],allegro_chain_[i]);
      pose_solver_[i].reset(new KDL::ChainFkSolverPos_recursive(allegro_chain_[i]));
      jac_solver_[i].reset(new KDL::ChainJntToJacSolver(allegro_chain_[i]));
      q_[i].resize(allegro_chain_[i].getNrOfJoints());
      J_[i].resize(allegro_chain_[i].getNrOfJoints());
   }
  k_gains.resize(6,6);
  k_jac.resize(4);
  k_gains.setIdentity();
  k_gains(3,3)=0.0;
  k_gains(4,4)=0.0;
  k_gains(5,5)=0.;
  // Initialize subscribers and publishers
  joint_pub_=n.advertise<sensor_msgs::JointState>("/allegro_hand_right/joint_cmd",1);

  control_pub_=n.advertise<std_msgs::Int8>("/allegro_hand_right/control_type",1);

  // initialize joint cmd names
  
  std::string names[]={"index_joint_0","index_joint_1","index_joint_2", "index_joint_3",
                   "middle_joint_0","middle_joint_1","middle_joint_2", "middle_joint_3",
                   "ring_joint_0","ring_joint_1","ring_joint_2", "ring_joint_3",
                   "thumb_joint_0","thumb_joint_1","thumb_joint_2", "thumb_joint_3"};
 
  std::vector<std::string> name_vect(names,names+sizeof(names)/sizeof(std::string)); //make it const if you can
  joint_cmd_.name=name_vect;
  joint_cmd_.effort.resize(16);
  control_type_.data=1;
  

  initialized_controller_=true;
  ROS_INFO("intialized controller");
}



