#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/JointState.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>

// Boost
#include <boost/shared_ptr.hpp>


// from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
// from pykdl_utils.kdl_kinematics import KDLKinematics

std::string EE_NAMES[4] = {"index_tip", "middle_tip", "ring_tip", "thumb_tip"};
enum FingerIdx { INDEX_IDX = 0,
                 MIDDLE_IDX,
                 RING_IDX,
                 THUMB_IDX,
                 NUM_FINGERS};
enum ControlType {TORQUE_CONTROL,
                  POSITION_CONTROL,
                  IK_CONTROL};

const int NUM_FINGER_JOINTS = 4;
const int DISPLAY_RATE = 1;

typedef KDL::Frame EEState;
typedef std::vector<EEState> EEStates;
typedef KDL::JntArray FingerJointState;
typedef std::vector<FingerJointState> FingerJointStates;
typedef KDL::ChainFkSolverPos_recursive FKSolver;
typedef KDL::ChainIkSolverVel_wdls IKSolver;
typedef boost::shared_ptr<FKSolver> FKSolverPtr;
typedef boost::shared_ptr<IKSolver> IKSolverPtr;

using std::string;

// Debugs
// #define DEBUG_FK_IK 1

class HandSimpleModel
{
 public:
  /**
   * Simple model of the hand hand kinematics and controls
   * Assume following state and action vectors
   *
   * @param urdf_file_name model file to load
   */
  HandSimpleModel(ros::NodeHandle n, string urdf_file_name, ControlType control_type=IK_CONTROL) :
      n_(n), control_type_(control_type), q_min_(NUM_FINGER_JOINTS), q_max_(NUM_FINGER_JOINTS)
  {
    // Load KDL trees for each hand finger
    if (!kdl_parser::treeFromFile(urdf_file_name, hand_tree_)){
      ROS_ERROR("Failed to construct kdl tree");
    }
    // Setup Joint Limits
    for (int i = 0; i < NUM_FINGER_JOINTS; ++i)
    {
      q_min_(i) = -2*M_PI;
      q_max_(i) = 2*M_PI;
    }

    // Set task space weights
    Eigen::MatrixXd weights_ts(6,6);
    for (int i = 0; i < 3; ++i)
    {
      weights_ts(i,i) = 1.0;
      weights_ts(i+3, i+3) = 0.003;
    }

    // TODO: Get this name better, but KDL doesn't easily support this
    //string base_link = hand_tree_.getRootSegment();//->segment.getName();
    base_link_ = "palm_link";
    for (int i = 0; i < NUM_FINGERS; ++i)
    {
      string tip_name = EE_NAMES[i];
      KDL::Chain fc;
      hand_tree_.getChain(base_link_, tip_name, fc);
      finger_chains_.push_back(fc);
      fk_.push_back(FKSolverPtr(new FKSolver(fc)));
      ik_.push_back(IKSolverPtr(new IKSolver(fc)));
      ik_.back()->setLambda(0.003);
      ik_.back()->setWeightTS(weights_ts);

      ROS_INFO_STREAM("Link " <<  i <<  " num joints: "  << fc.getNrOfJoints());
    }
    // setup subscribers and publishers
    joint_sub_ = n_.subscribe("/allegro_hand_right/joint_states", 100,
                              &HandSimpleModel::jointStateCB, this);

    joint_cmd_pub_ = n_.advertise<sensor_msgs::JointState>("/allegro_hand_right/joint_cmd",
                                                           100);
    torque_cmd_pub_ = n_.advertise<sensor_msgs::JointState>("/allegro_hand_right/torque_cmd",
                                                            100);
    control_type_pub_ = n_.advertise<std_msgs::Char>("/allegro_hand_right/control_type", 100);

    // # TODO: Get / set this from the lib_cmd?
    // self.active_controller = None
    sign_ = 1;
    std_msgs::Char control_cmd;
    control_cmd.data='p';
    control_type_pub_.publish(control_cmd);
  }

  void run(void)
  {
    for (int i = 0; i < finger_chains_.size(); ++i)
    {
      int k = finger_chains_[i].getNrOfJoints();
    }
    ros::spin();
  }

  /**
   * Method to convert joint positions to task coordinates
   *
   * @param finger_qs joint state of a finger chain
   *
   * @return end effector state of the finger chain
   */
  EEStates FK(FingerJointStates finger_qs)
  {
    EEStates X;
    for (int i=0; i < finger_qs.size(); ++i) {
      string target_link = EE_NAMES[i];
      EEState fi_x;
      fk_[i]->JntToCart(finger_qs[i], fi_x);
      X.push_back(fi_x);
    }
    return X;
  }

  /**
   * Perform forward kinematics for a single finger
   *
   * @param q Joint configuration of the finger
   * @param finger which finger to perform FK on
   *
   * @return Position of the finger end effector
   */
  EEState FK(FingerJointState q, FingerIdx finger)
  {
    EEState x;
    fk_[finger]->JntToCart(q, x);
    return x;
  }

  /**
   * Get inverse kinematics for desired finger poses of all fingers
   *
   * @param fingers_desired a vector of desired finger tip poses in order of finger_chains[]
   * @param finger
   *
   * @return a list of lists of pyhton arrays of finger joint configurations
   */
  FingerJointStates IK(EEStates fingers_desired)
  {
    FingerJointStates q_desired_fingers;
    for (int i = 0; i < fingers_desired.size(); i++)
    {
      // FingerJointState q_i(finger_chains_[i].getNrOfJoints());
      // FingerJointState q_init(finger_chains_[i].getNrOfJoints());
      // //Maximum 100 iterations, stop at accuracy 1e-6
      // KDL::ChainIkSolverPos_NR_JL iksolver(finger_chains_[i], q_min_, q_max_,
      //                                      *fk_[i],*ik_[i],100,1e-6);
      // iksolver.CartToJnt(q_init, fingers_desired[i], q_i);
      q_desired_fingers.push_back(IK(fingers_desired[i], (FingerIdx)i));
    }
    return q_desired_fingers;
  }

  /**
   * Perform IK for a single finger
   *
   * @param x_d Desired end effector position of the finger
   * @param finger Which finger to perform IK on
   *
   * @return Desired joint configuration of the finger
   */
  FingerJointState IK(EEState x_d, FingerIdx finger)
  {
    FingerJointState q_d(finger_chains_[finger].getNrOfJoints());
    FingerJointState q_init(finger_chains_[finger].getNrOfJoints());
    //Maximum 100 iterations, stop at accuracy 1e-6
    KDL::ChainIkSolverPos_NR_JL iksolver(finger_chains_[finger], q_min_, q_max_,
                                         *fk_[finger],*ik_[finger],100,1e-6);
    iksolver.CartToJnt(q_init, x_d, q_d);
    return q_d;
  }

  /**
   * Callback method on the current hand state
   *
   * @param cur_joint_state current joint values for hand
   */
  void jointStateCB(sensor_msgs::JointState cur_joint_state)
  {
    if (cur_joint_state.header.seq % DISPLAY_RATE == 0)
    {
      switch(control_type_)
      {
        case TORQUE_CONTROL:
          testTorqueControl(cur_joint_state);
          break;
        case IK_CONTROL:
          testIKControl(cur_joint_state);
          break;
        case POSITION_CONTROL:
          testPositionControl(cur_joint_state);
          break;
      }
    }
  }

  /**
   * Simple method to solve for IK solutions and then send PD position control
   * to the hand. Serves to show easy interfacing with the hand
   *
   * @param cur_joint_state  Current joint state of the hand
   */
  void testIKControl(sensor_msgs::JointState cur_joint_state)
  {
    // Parse current state into separate finger chains
    FingerJointStates finger_qs;
    for (int i = 0; i < NUM_FINGERS; ++i)
    {
      FingerJointState fi_q(NUM_FINGER_JOINTS);
      for (int j = 0; j < NUM_FINGER_JOINTS; ++j)
      {
        fi_q(j) = cur_joint_state.position[i*NUM_FINGER_JOINTS+j];
      }
      finger_qs.push_back(fi_q);
    }

    // Testing FK and IK functions on each finger chain
    EEStates X = FK(finger_qs);
    FingerJointStates Q = IK(X);

    // Test PD position control
    sensor_msgs::JointState q_desired;
    // TODO: Remove joint names that aren't to do with the hand
    q_desired.name = cur_joint_state.name;
    q_desired.position.resize(q_desired.name.size(), 0.0);
    // Translate IK result back into the message format
    for (int i = 0; i < NUM_FINGERS; ++i)
    {
      for (int j = 0; j < NUM_FINGER_JOINTS; ++j)
      {
        q_desired.position[i*4+j] = Q[i](j);
      }
#ifdef DEBUG_FK_IK
      ROS_INFO_STREAM("Finger " << EE_NAMES[i]);
      ROS_INFO_STREAM("Cur joints\n [" << finger_qs[i](0) << ", "<< finger_qs[i](1) << ", "<<
          finger_qs[i](2) << ", "<< finger_qs[i](3) << "]");
      ROS_INFO_STREAM("FK results\n " << X[i]);
      ROS_INFO_STREAM("IK results\n [" << Q[i](0) << ", "<< Q[i](1) << ", "<<
          Q[i](2) << ", "<< Q[i](3) << "]\n");
#endif // DEBUG_FK_IK
    }

    // TODO: Get desired thumb pose
    KDL::Vector x_th_pos_des(0.03, 0.0, 0.0);
    x_th_pos_des(0) += (cur_joint_state.header.seq%2)*0.3;
    KDL::Rotation x_th_rot_des(1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0);
    EEState x_th_des(x_th_rot_des, x_th_pos_des);
    FingerJointState q_des = IK(x_th_des, THUMB_IDX);
    ROS_INFO_STREAM("q_des = [" << q_des(0) << ", " << q_des(1) << ", "
                                << q_des(2) << ", " << q_des(3) << "] "
                                << " for x_des = " << x_th_des );
    for (int i = 0; i < NUM_FINGER_JOINTS; ++i)
    {
      q_desired.position[THUMB_IDX*NUM_FINGER_JOINTS+i] = q_des(i);
    }

    joint_cmd_pub_.publish(q_desired);
  }


  void testPositionControl(sensor_msgs::JointState cur_joint_state)
  {
    // HACK: Set values directly here
    // Test PD position control
    sensor_msgs::JointState q_desired;
    // TODO: Remove joint names that aren't to do with the hand
    q_desired.name = cur_joint_state.name;
    q_desired.position = cur_joint_state.position;
    if (cur_joint_state.header.seq % 50 == 0)
    {
      q_desired.position[0] = 0.0;
      sign_ *= -1;
    }
    else
    {
      q_desired.position[0] += 0.2;
    }
    for (int i = 0; i < q_desired.name.size(); ++i)
    {
      q_desired.position[i] += sign_*0.2;
      ROS_INFO_STREAM(cur_joint_state.name[i] << " " << q_desired.position[i]);
    }
    ROS_INFO_STREAM("");

    joint_cmd_pub_.publish(q_desired);

  }

  void testTorqueControl(sensor_msgs::JointState cur_joint_state)
  {
  //     '''
  //     Simple feedback control test of the torque command for the allegro hand
  //     '''
  //     FingerJointStates finger_qs;
  //     for fi in xrange(_NUM_FINGERS):
  //         fi_q = []
  //         for j in xrange(_NUM_FINGER_JOINTS):
  //             fi_q.push_back(cur_joint_state.position[fi*4+j])
  //         finger_qs.push_back(np.array(fi_q))

  //     # Convert desired force to desired torques (tau = J^t F )
  //     F_desired = np.matrix([0, 1, -0.4, 0, 0, 0]).T
  //     J = self.finger_chains[_INDEX_IDX].jacobian(finger_qs[_INDEX_IDX])
  //     tau_d = J.T*F_desired
  //     print 'F_d = ',F_desired
  //     print 'J = ',J
  //     print 'tau = ',tau_d

  //     # TODO: Convert array response into tau_desired mode
  //     tau_desired = sensor_msgs::JointState()
  //     tau_desired.name = list(cur_joint_state.name[:])
  //     tau_desired.effort = np.zeros(len(tau_desired.name))

  //     start_idx = _INDEX_IDX*_NUM_FINGER_JOINTS
  //     end_idx = start_idx + _NUM_FINGER_JOINTS
  //     tau_desired.effort[start_idx:end_idx] = tau_d.flatten()[0,start_idx:end_idx]
  //     # print cur_joint_state,'\n'
  //     print tau_desired,'\n'

  //     self.torque_cmd_pub.publish(tau_desired)
  //     }
  }

 private:
  ros::NodeHandle n_;
  ros::Publisher joint_cmd_pub_;
  ros::Publisher torque_cmd_pub_;
  ros::Publisher control_type_pub_;
  ros::Subscriber joint_sub_;
  std::vector<KDL::Chain> finger_chains_;
  std::vector<FKSolverPtr> fk_;
  std::vector<IKSolverPtr> ik_;
  KDL::Tree hand_tree_;
  string base_link_;
  int sign_;
  ControlType control_type_;
  KDL::JntArray q_min_;
  KDL::JntArray q_max_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_ik_control_node");
  ros::NodeHandle n;

  string urdf_file_name = ros::package::getPath("allegro_hand_description") +
                          "/allegro_hand_description_right.urdf";

  HandSimpleModel model(n, urdf_file_name);
  model.run();

  return 0;
}
