#include "in_hand_task_controllers/in_grasp/mpc_framework.h"
#include <fstream>


// This is the main control loop, declared in the mpc_framework.h file and defined here for flexibility
void hand_controller::mpcController::run_controller()
{
  //
  ros::Rate loop_rate(30);
  int T=99;
  int t=0;
  // get initial rigid frame:
  get_object_pose();
  js_gain=1.0;
  obj_gain=10.0;
  float obj_err_data,final_obj_err_data;
  obj_err_data=0.0;
  while (n_->ok() && t<T)
  {
    updateState();
    get_object_pose();
    // get rigid transformation between object pose and rigid finger
    Eigen::Matrix<double,4,4> thumb_T_obj;
    Eigen::Affine3d a;
    tf::transformKDLToEigen(curr_rigid_frame,a);
    thumb_T_obj=a.matrix();
    
    
    // Compute new command
    // Joint trajectory tracking:
    Map<Eigen::VectorXd> curr_js(&joint_state_.position[0],joint_state_.position.size());
    
    Eigen::VectorXd js_ff=j_des[t+1]-j_des[t];
    
    Eigen::VectorXd js_fb=1.0*(j_des[t]-curr_js);
    
    //cerr<<"JOINT ERROR: "<<js_fb.lpNorm<1>()<<endl;
    js_fb*=js_gain;

    // Object pose tracking:


    // jacobian of thumb:
    vector<double> q(&curr_js[12],&curr_js[12]+4);
    Eigen::MatrixXd J=hand_kdl->getJacobian(3,q);

    // observed rigid transform thumb_T_obj:

    // Desired object pose:

    // Get KDL frame of current object pose:
     
    // Current pose from planner:
    vector<double> q_des(&curr_js[12],&curr_js[12]+4);
    
    //Add next trajectory to q_des
    for(int i=0;i<4;++i)
    {
      q_des[i]+=j_des[t+1][3*4+i]-j_des[t][3*4+i];
    }
    
    Eigen::MatrixXd f_mat=hand_kdl->getFKEigen(3,q_des);
    
    Eigen::MatrixXd cur_obj_pose=f_mat;
    
    //
    vector<double> pose=hand_kdl->frame_to_pose_(cur_obj_pose,true);
    KDL::Rotation r_obj=KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
    KDL::Vector t_obj(pose[0],pose[1],pose[2]);
    KDL::Frame cur_obj(r_obj,t_obj);

    // desired f_tip pose:
    Eigen::Affine3d a_;
    tf::transformKDLToEigen(obj_pose_des[t+1],a_);

    //cerr<<obj_er_v.transpose()<<endl;
    Eigen::MatrixXd obj_mat=a_.matrix();

    Eigen::MatrixXd des_f_mat=obj_mat*thumb_T_obj.inverse();
    
    vector<double> f_des_pose=hand_kdl->frame_to_pose_(des_f_mat,true);
    KDL::Rotation r_obj_des=KDL::Rotation::RPY(f_des_pose[3],f_des_pose[4],f_des_pose[5]);
    KDL::Vector t_obj_des(f_des_pose[0],f_des_pose[1],f_des_pose[2]);
    KDL::Frame cur_obj_des(r_obj_des,t_obj_des);

    
    
    KDL::Twist obj_err=KDL::diff(cur_obj_des,cur_obj);
    
    Eigen::Matrix<double,6,1> obj_err_eigen;
    
    tf::twistKDLToEigen(obj_err,obj_err_eigen);
    Eigen::VectorXd obj_er_v=obj_err_eigen.col(0);
    //cerr<<obj_er_v.transpose()<<endl;


    
    vector<double> err_obj(&obj_er_v[0],&obj_er_v[0]+6);
    Eigen::MatrixXd err_mat=hand_kdl->pose_to_frame(err_obj);
    
    
    
    
    // error:
    //Eigen::MatrixXd f_err_=err_mat*thumb_T_obj.inverse();
    Eigen::MatrixXd f_err_=err_mat;
    //cerr<<f_err_<<endl;
    vector<double> f_err=hand_kdl->frame_to_pose_(f_err_,true);
    //
    f_err[3]*=0.001;
    f_err[4]*=0.001;
    f_err[5]*=0.001;
    Map<Eigen::VectorXd> f_err_v(&f_err[0],f_err.size());
    
    Eigen::VectorXd obj_fb=-1.0*J.transpose()*(f_err_v);
    for(int i=0;i<4;++i)
    {
      js_fb[4*3+i]=obj_gain*obj_fb[i];
    }
    
    // publish trajectory:
    Eigen::VectorXd js_cmd=curr_js+js_ff+js_fb;
    obj_err_data+=f_err_v.lpNorm<1>();
    final_obj_err_data=f_err_v.lpNorm<1>();
    //cerr<<"OBJ ERROR: "<<<<endl;   
    publish_joints(js_cmd);
    t++;
    loop_rate.sleep();
   
  }
  cerr<<"Controller done!"<<endl;
  vector<double> err_arr;
  err_arr.emplace_back(obj_err_data);
  err_arr.emplace_back(final_obj_err_data);
  //err_=err_arr;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"one_step_mpc_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
  hand_controller::mpcController robot_node;
  robot_node.init(n,"/allegro_hand_right");
  tf::TransformListener listen;
  robot_node.init_controller(listen);
  

  while(n.ok())
  {
    // controller is called inside the service, this loop just looks for a new service request
    ros::spinOnce();
    loop_rate.sleep();
  }


}

