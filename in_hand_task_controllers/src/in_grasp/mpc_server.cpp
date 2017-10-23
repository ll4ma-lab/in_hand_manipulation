#include "in_hand_task_controllers/in_grasp/mpc_framework.h"
#include <fstream>


// This is the main control loop, declared in the mpc_framework.h file and defined here for flexibility
void hand_controller::mpcController::run_controller()
{
  //
  ros::Rate loop_rate(100);
  int T=99;
  int t=0;
  // open files:
  ofstream data_file;
  string filename;
  if (!n_->getParam("/mpc/log_file",filename))
  {
    ROS_ERROR("Failed to read mpc_log file!!!");
    ros::shutdown();
  }

  data_file.open(filename);
  for(int i=0; i<16;++i)
  {
    data_file<<"des_j"<<i<<",";
  }
  
  for(int i=0; i<16;++i)
  {
    data_file<<"obs_j"<<i<<",";
  }
  for(int i=0; i<16;++i)
  {
    data_file<<"ff_j"<<i<<",";
  }
  for(int i=0; i<16;++i)
  {
    data_file<<"fb_j"<<i<<",";
  }
  data_file<<endl;
  while (n_->ok() && t<T)
  {
    updateState();
    //get_object_pose();

    // get rigid transformation between object pose and rigid finger
    
    // Compute new command
    // Joint trajectory tracking:
    Map<Eigen::VectorXd> curr_js(&joint_state_.position[0],joint_state_.position.size());
    
    Eigen::VectorXd js_ff=j_des[t+1]-j_des[t];
    
    Eigen::VectorXd js_fb=2.0*(j_des[t+1]-curr_js-js_ff);

    Eigen::VectorXd js_cmd=j_des[t]+js_ff+js_gain*js_fb;
    
    // Object pose tracking:
    
    // convert rigid frame to T mat:
    Eigen::Matrix<double,4,4> rigid_T;
    Eigen::Affine3d a;
    tf::transformKDLToEigen(curr_rigid_frame,a);
    rigid_T=a.matrix();

   
    //Computing fb err:
    KDL::Twist obj_err=KDL::diff(obj_pose_des[t],curr_obj_pose);

    Eigen::Matrix<double,6,1> eigen_err;
    tf::twistKDLToEigen(obj_err,eigen_err);
    Eigen::MatrixXd obj_err_mat=hand_kdl->pose_to_frame(eigen_err);

    vector<double> pose=hand_kdl->frame_to_pose_(obj_err_mat*rigid_T,true);
    Map<Eigen::VectorXd> obj_fb(&pose[0],pose.size());
    
    
    
    /*
    // Computing ff term:
    obj_err=KDL::diff(obj_pose_des[t+1],obj_pose_des[t]);
    
    // Get object error:
    tf::twistKDLToEigen(obj_err,eigen_err);
    obj_err_mat=hand_kdl->pose_to_frame(eigen_err);
    pose=hand_kdl->frame_to_pose_(obj_err_mat*rigid_T,true);
    Map<Eigen::VectorXd> obj_ff(&pose[0],pose.size());


    // Converting to joint command:
    vector<double> q(&curr_js[12],&curr_js[12+4]);
    Eigen::MatrixXd J=hand_kdl->getJacobian(3,q);

    Eigen::VectorXd thumb_fb=J.transpose()*(obj_fb+obj_ff);
    */
    
    // publish trajectory:
    publish_joints(js_cmd);
    for(int i=0;i<16;++i)
    {
      data_file<<j_des[t][i]<<",";
    }
    for(int i=0;i<16;++i)
    {
      data_file<<curr_js[i]<<",";
    }
    for(int i=0;i<16;++i)
    {
      data_file<<js_ff[i]<<",";
    }

    for(int i=0;i<16;++i)
    {
      data_file<<js_fb[i]<<",";
    }

    data_file<<endl;
    t++;
    loop_rate.sleep();
  }
  data_file.close();
  cerr<<"Controller done!"<<endl;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"one_step_mpc_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
  hand_controller::mpcController robot_node;
  robot_node.init(n,"/allegro_hand_right");
  robot_node.init_controller();


  while(n.ok())
  {
    // controller is called inside the service, this loop just looks for a new service request
    ros::spinOnce();
    loop_rate.sleep();
  }


}



// Get desired joint trajectory, desired object pose trajectory

// Compute feedforward delta_theta

// while loop:(t<T) T=interpolated timesteps

// Compute transformation matrix between thumb and object

// Compute feedback delta_theta

// command robot

