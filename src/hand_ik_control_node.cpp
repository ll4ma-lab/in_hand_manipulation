#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/JointState.h>
 #include <kdl_parser/kdl_parser.hpp>

// from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
// from pykdl_utils.kdl_kinematics import KDLKinematics

std::string EE_NAMES[4] = {"index_tip", "middle_tip", "ring_tip", "thumb_tip"};
enum FingerIdx { INDEX_IDX = 0,
                 MIDDLE_IDX,
                 RING_IDX,
                 THUMB_IDX,
                 NUM_FINGERS};

#define NUM_FINGER_JOINTS  4
// #define TEST_TORQUE_CONTROL 1
#define TEST_IK_CONTROL 1
#define DISPLAY_RATE 1

// TODO: Replace this with the correct type
typedef std::vector<float> EEState;
typedef std::vector<EEState> EEStates;
typedef std::vector<float> FingerJointState;
typedef std::vector<FingerJointState> FingerJointStates;
using std::string;

class HandSimpleModel
{
 public:
  /**
   * Simple model of the hand hand kinematics and controls
   * Assume following state and action vectors
   *
   * @param urdf_file_name model file to load
   */
  HandSimpleModel(ros::NodeHandle n, string urdf_file_name) : n_(n)
  {
    // Load KDL trees for each hand finger
    if (!kdl_parser::treeFromFile(urdf_file_name, hand_tree_)){
      ROS_ERROR("Failed to construct kdl tree");
    }
    ROS_INFO_STREAM("\nNum segements:" << hand_tree_.getNrOfSegments() << "\n");

    // task_space_ik_weights = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]).tolist();

    // TODO: Get this name better, but KDL doesn't easily support this
    //string base_link = hand_tree_.getRootSegment();//->segment.getName();
    base_link_ = "palm_link";
    for (int i = 0; i < NUM_FINGERS; ++i)
    {
      string tip_name = EE_NAMES[i];
      KDL::Chain fc;
      hand_tree_.getChain(base_link_, tip_name, fc);
      // fc.set_wdls_weights(weights_ts = task_space_ik_weights)
      // fc._ik_wdls_kdl.setLambda(0.003)
      finger_chains_.push_back(fc);
      ROS_INFO_STREAM("Link " <<  i <<  " num joints: "  << fc.getNrOfJoints());
    }
    // setup subscribers and publishers
    joint_sub_ = n_.subscribe("/allegro_hand_right/joint_states", 100, &HandSimpleModel::jointStateCB, this);

    joint_cmd_pub_ = n_.advertise<sensor_msgs::JointState>("/allegro_hand_right/joint_cmd", 100);
    torque_cmd_pub_ = n_.advertise<sensor_msgs::JointState>("/allegro_hand_right/torque_cmd", 100);
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
    // TODO: Change draw robot to use klampt / real robot / publish to rviz
    for (int i = 0; i < finger_chains_.size(); ++i)
    {
      int k = finger_chains_[i].getNrOfJoints();
      ROS_INFO_STREAM("Chain " << i << " has " << k << " joints.");
      ROS_INFO_STREAM("From " << finger_chains_[i].getSegment(0).getName() << " to " <<
                      finger_chains_[i].getSegment(k-1).getName());
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
      EEState fi_x; // = finger_chains_[i].FK(finger_qs[i]);
      X.push_back(fi_x);
    }
    return X;
  }

  /**
   * Get inverse kinematics for desired finger poses of all fingers
   *
   * @param fingers_desired a vector of desired finger tip poses in order of finger_chains[]
   * @param finger
   *
   * @return a list of lists of pyhton arrays of finger joint configurations
   */
  FingerJointStates IK(EEStates fingers_desired, FingerIdx finger)
  {
    FingerJointStates q_desired_fingers;
    for (int i = 0; i < fingers_desired.size(); i++)
    {
      // q_desired_fingers.push_back(finger_chains_[i].inverse_wdls(fingers_desired[i]));
      // q_desired_fingers.push_back(finger_chains_[i].inverse(fingers_desired[i]));
    }
    return q_desired_fingers;
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
#ifdef TEST_TORQUE_CONTROL
      testTorqueControl(cur_joint_state);
#else
#ifdef TEST_IK_CONTROL
      testIKControl(cur_joint_state);
#endif // TEST_IK_CONTROL
#endif // TEST_TORQUE_CONTORL
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
      FingerJointStates fi_q;
      //         for (int j = 0; j < NUM_FINGER_JOINTS; ++j)
      //             fi_q.push_back(cur_joint_state.position[fi*4+j])
      //         finger_qs.push_back(np.array(fi_q))
    }
    // Testing FK and IK functions on each finger chain
    // X = FK(finger_qs);
    // ROS_INFO_STREAM("X= " << X);
    // Q = IK(X);
    // ROS_INFO_STREAM("Q= ",Q);

    // Test PD position control
    sensor_msgs::JointState q_desired;
    // TODO: Remove joint names that aren't to do with the hand
    q_desired.name = cur_joint_state.name;
    q_desired.position = cur_joint_state.position;

    // HACK: Set values directly here
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
    // q_desired.position[13] += sign_*0.15;
    // q_desired.position[14] += sign_*0.1;
    // q_desired.position[15] += sign_*0.05;
    // ROS_INFO_STERAM("Joint error = " <<  q_desired.position - cur_joint_state.position);
    for (int i = 0; i < NUM_FINGERS; ++i)
    {
      // ROS_INFO_STREAM(EE_NAMES[i] <<  " P =\n" <<  X[i][0] << "\nR =\n" <<  X[i][1]);
      // ROS_INFO_STREAM("Q_solved =\n" << Q[i]);
      // int start_i = i*4;
      // int end_i = start_i+4;
      // print 'Q_cur =\n', cur_joint_state.position[start_i:end_i]
    }

    // Get desired thumb pose
    // x_th_des = np.matrix([[0.03],
    //                       [0.0],
    //                       [0.0]])
    // r_th_des = np.matrix(np.eye(3))
    // X_th_des = (x_th_des, r_th_des)

    // q_des = finger_chains_[INDEX_IDX].inverse_wdls(X_th_des)
    //  print q_des
    // print ' '

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
  KDL::Tree hand_tree_;
  string base_link_;
  int sign_;
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
