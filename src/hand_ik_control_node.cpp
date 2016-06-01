#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

// from urdf_parser_py.urdf import Robot
// from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
// from pykdl_utils.kdl_kinematics import KDLKinematics

// from std_msgs.msg import String

// import numpy as np
// import matplotlib.pyplot as plotter
// import time

#define DISPLAY_RATE 100

using std::string;

string EE_NAMES[4] = {"index_tip", "middle_tip", "ring_tip", "thumb_tip"};


enum FingerIdx { INDEX_IDX = 0,
                 MIDDLE_IDX,
                 RING_IDX,
                 THUMB_IDX,
                 NUM_FINGERS};
#define _NUM_FINGER_JOINTS  4
#define TEST_TORQUE_CONTROL false

// #define TEST_IK_CONTROL true

// TODO: Replace this with the correct type
typedef std::vector<float> EEState;
typedef std::vector<EEState> EEStates;
typedef std::vector<sensor_msgs::JointState> JointStates;

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
    // urdf_file = file(urdf_file_name, 'r');
    // robot_ = Robot.from_xml_string(urdf_file.read());
    // urdf_file.close();
    // tree_ = kdl_tree_from_urdf_model(self.robot);
    // ROS_INFO_STREAM("\nNum segements:" << tree_.getNrOfSegments() << "\n");

    // task_space_ik_weights = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]).tolist();

    // self.base_link = self.robot.get_root();
    //self.base_link = 'palm_link';
    // self.finger_chains = []
    //     for i, ee_name in enumerate(_EE_NAMES):
    //         fc = KDLKinematics(self.robot, self.base_link, ee_name)
    //         fc.set_wdls_weights(weights_ts = task_space_ik_weights)
    //         fc._ik_wdls_kdl.setLambda(0.003)
    //         self.finger_chains.push_back(fc)
    //         print 'Link', i, 'num joints:',fc.chain.getNrOfJoints()

    // setup subscribers and publishers
    joint_sub_ = n_.subscribe("/allegroHand_0/joint_states", 100, &HandSimpleModel::jointStateCB, this);

    joint_cmd_pub_ = n_.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd", 100);
    torque_cmd_pub_ = n_.advertise<sensor_msgs::JointState>("/allegroHand_0/torque_cmd", 100);
    control_type_pub_ = n_.advertise<std_msgs::String>("/allegroHand_0/lib_cmd", 100);

    // # TODO: Get / set this from the lib_cmd?
    // self.active_controller = None
  }

  void run(void)
  {
    // TODO: Change draw robot to use klampt / real robot / publish to rviz
    ros::spin();
  }

  /**
   * Method to convert joint positions to task coordinates
   *
   * @param finger_qs joint state of a finger chain
   *
   * @return end effector state of the finger chain
   */
  EEStates FK(JointStates finger_qs)
  {
    EEStates X;
    for (int i=0; i < finger_qs.size(); ++i) {
      string target_link = EE_NAMES[i];
      // EEstate fi_x = finger_chains_[i].FK(finger_qs[i]);
      // X.push_back(fi_x);
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
  JointStates IK(EEStates fingers_desired, FingerIdx finger)
  {
    JointStates q_desired_fingers;
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

  void testIKControl(sensor_msgs::JointState cur_joint_state)
  {
  //     '''
  //     Simple method to solve for IK solutions and then send PD position control
  //     to the hand. Serves to show easy interfacing with the hand
  //     '''
  //     # Parse current state into separate finger chains
  //     finger_qs = []
  //     for fi in xrange(_NUM_FINGERS):
  //         fi_q = []
  //         for j in xrange(_NUM_FINGER_JOINTS):
  //             fi_q.push_back(cur_joint_state.position[fi*4+j])
  //         finger_qs.push_back(np.array(fi_q))

  //     # Testing FK and IK functions on each finger chain
  //     X = self.FK(finger_qs)
  //     print 'X=',X
  //     Q = self.IK(X)
  //     print 'Q=',Q
  //     # Test PD position control
  //     q_desired = sensor_msgs::JointState()
  //     q_desired.name = list(cur_joint_state.name[:])
  //     q_desired.position = list(cur_joint_state.position[:])

  //     q_desired.position[12] -= 0.25
  //     q_desired.position[13] -= 0.2
  //     q_desired.position[14] -= 0.1
  //     print np.array(q_desired.position) - np.array(cur_joint_state.position)
  //     print cur_joint_state.name
  //     print cur_joint_state.name[12]
  //     print 'q_desired=',q_desired.position
  //     for i in xrange(_NUM_FINGERS):
  //         print _EE_NAMES[i], 'P =\n', X[i][0],'\nR =\n', X[i][1]
  //         print 'Q_solved =\n', Q[i]
  //         start_i = i*4
  //         end_i = start_i+4
  //         print 'Q_cur =\n', cur_joint_state.position[start_i:end_i]


  //     # Get desired thumb pose
  //     x_th_des = np.matrix([[0.03],
  //                           [0.0],
  //                           [0.0]])
  //     r_th_des = np.matrix(np.eye(3))
  //     X_th_des = (x_th_des, r_th_des)

  //     # try:
  //     #     q_des = self.finger_chains[_INDEX_IDX].inverse_wdls(X_th_des)
  //     #     print q_des
  //     #     print ' '
  //     # except (RuntimeError, TypeError, NameError):
  //     #     pass

  //     self.joint_cmd_pub.publish(q_desired)
  }

  void testTorqueControl(sensor_msgs::JointState cur_joint_state)
  {
  //     {
  //     '''
  //     Simple feedback control test of the torque command for the allegro hand
  //     '''
  //     JointStates finger_qs;
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
