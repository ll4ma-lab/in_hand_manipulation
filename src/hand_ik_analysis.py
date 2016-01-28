#!/usr/bin/env python

#import rospy
import roslib
import rospy
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import numpy as np
import matplotlib.pyplot as plotter
import scipy.optimize as opt
from math import sin, cos, pi
import time

_DISPLAY_RATE = 100
_EE_NAMES = ['index_tip', 'middle_tip', 'ring_tip', 'thumb_tip']
_INDEX_IDX = 0
_MIDDLE_IDX = 1
_RING_IDX = 2
_THUMB_IDX = 3
class HandSimpleModel:
    def __init__(self, urdf_file_name):
        '''
        Simple model of the hand hand kinematics and controls
        Assume following state and action vectors
        urdf_file_name - model file to load
        '''
        rospy.init_node('hand_model_node')

        # Load KDL trees for each hand finger
        urdf_file = file(urdf_file_name, 'r')
        self.robot = Robot.from_xml_string(urdf_file.read())
        urdf_file.close()
        self.tree = kdl_tree_from_urdf_model(self.robot)
        print '\nNum segements:',self.tree.getNrOfSegments(),'\n'


        task_space_ik_weights = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]).tolist()

        self.base_link = self.robot.get_root()
        # self.base_link = 'palm_link'
        self.finger_chains = []
        for i, ee_name in enumerate(_EE_NAMES):
            fc = KDLKinematics(self.robot, self.base_link, ee_name)
            fc.set_wdls_weights(weights_ts = task_space_ik_weights)
            fc._ik_wdls_kdl.setLambda(0.003)
            self.finger_chains.append(fc)
            print 'Link', i, 'num joints:',fc.chain.getNrOfJoints()

        # setup subscribers and publishers
        rospy.Subscriber('/allegroHand_0/joint_states', JointState,
                         self.joint_state_cb)

        self.joint_cmd_pub = rospy.Publisher('/allegroHand_0/joint_cmd', JointState,
                                             queue_size=100)
        # TODO: Get / set this from the lib_cmd
        self.active_controller = None
        self.control_type_pub = rospy.Publisher('/allegroHand_0/lib_cmd', String, queue_size=100)

    def FK(self, finger_qs):
        '''
        Method to convert joint positions to task coordinates
        '''
        X = []
        for i, fi_q in enumerate(finger_qs):
            target_link = _EE_NAMES[i]
            fi_x = self.finger_chains[i].FK(fi_q)
            X.append(fi_x)
        return X

    def IK(self, fingers_desired, finger=None):
        '''
        Get inverse kinematics for desired finger poses of all fingers
        fingers_desired - a list of desired finger tip poses in order of finger_chains[]
        returns - a list of lists of pyhton arrays of finger joint configurations
        TODO: Allow for single finger computation (named fingers)
        '''
        q_desired_fingers = []
        if finger is not None:
            # TODO: do this
            return None
        for i, f_d in enumerate(fingers_desired):
            q_desired_fingers.append(self.finger_chains[i].inverse(f_d))
        return q_desired_fingers

    def joint_state_cb(self, cur_joint_state):
        # Parse current state into separate finger chains
        finger_qs = []
        for fi in xrange(4):
            fi_q = []
            for j in xrange(4):
                fi_q.append(cur_joint_state.position[fi*4+j])
            finger_qs.append(np.array(fi_q))
        # Testing FK and IK functions on each finger chain
        X = self.FK(finger_qs)
        Q = self.IK(X)
        q_desired = JointState()
        q_desired.name = list(cur_joint_state.name[:])
        q_desired.position = list(cur_joint_state.position[:])

        if cur_joint_state.header.seq % _DISPLAY_RATE == 0:
            q_desired.position[12] += 0.1
            q_desired.position[13] -= 0.1
            q_desired.position[14] -= 0.1
            print np.array(q_desired.position) - np.array(cur_joint_state.position)
            print cur_joint_state.name
            print cur_joint_state.name[12]
            print 'q_desired=',q_desired.position
            for i in xrange(4):
                # print _EE_NAMES[i], 'P =\n', X[i][0],'\nR =\n', X[i][1]
                # print 'Q_solved =\n', Q[i]
                start_i = i*4
                end_i = start_i+4
                # print 'Q_cur =\n', cur_joint_state.position[start_i:end_i]


            # Get desired thumb pose
            x_th_des = np.matrix([[0.03],
                                  [0.0],
                                  [0.0]])
            r_th_des = np.matrix(np.eye(3))
            X_th_des = (x_th_des, r_th_des)

            # try:
            #     q_des = self.finger_chains[_INDEX_IDX].inverse_wdls(X_th_des)
            #     print q_des
            #     print ' '
            # except (RuntimeError, TypeError, NameError):
            #     pass

            self.joint_cmd_pub.publish(q_desired)


    def Q_to_finger_joints(self, Q):
        Q_f1 = np.array([Q[-1], Q[0], Q[2]])
        Q_f2 = np.array([-Q[-1], Q[1], Q[3]])
        return (Q_f1, Q_f2)

    def forward_prediction(self, Q_k, U_k):
        '''
        Predict estimate of hand pose at next timestep based on current state Q and current command U
        '''
        Q_k_plus_1 = Q_k.copy()

        # Increment the proximal joints with the commanded joint change
        Q_k_plus_1[0] += U[0]
        Q_k_plus_1[2] += U[1]

        return Q_k_plus_1

    def run(self):
        # TODO: Change draw robot to use klampt / real robot / publish to rviz
        rospy.spin()

def main():
    urdf_file_name = roslib.packages.get_pkg_dir('allegro_hand_description') + '/allegro_hand_description_right.urdf'
    model = HandSimpleModel(urdf_file_name)

    model.run()
    return model

if __name__ == '__main__':
    main()
