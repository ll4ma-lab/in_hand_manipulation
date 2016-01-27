#!/usr/bin/env python

#import rospy
import roslib
import rospy
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import JointState

import numpy as np
import matplotlib.pyplot as plotter
import scipy.optimize as opt
from math import sin, cos, pi
import time

_DISPLAY_RATE = 100
_EE_NAMES = ['index_tip', 'middle_tip', 'ring_tip', 'thumb_tip']
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

        # TODO: Get list of base link for all fingers
        self.base_link = self.robot.get_root()
        # self.base_link = 'palm_link'
        self.finger_chains = []
        for i, ee_name in enumerate(_EE_NAMES):
            fc = KDLKinematics(self.robot, self.base_link, ee_name)
            self.finger_chains.append(fc)
            print 'Link', i, 'num joints:',fc.chain.getNrOfJoints()
            # TODO: Set joint limits here

        # setup subscribers and publishers
        rospy.Subscriber('/allegroHand_0/joint_states', JointState,
                         self.joint_state_cb)

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

    def IK(self, fingers_desired):
        q_desired_fingers = []
        for i, f_d in enumerate(fingers_desired):
            q_desired_fingers.append(self.finger_chains[i].inverse(f_d))
        # TODO: send desired IK to joint trajectory controller
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
        if cur_joint_state.header.seq % _DISPLAY_RATE == 0:
            for i in xrange(4):
                print _EE_NAMES[i], 'P =\n', X[i][0],'\nR =\n', X[i][1]
                print 'Q_solved =\n', Q[i]
                start_i = i*4
                end_i = start_i+4
                print 'Q_cur =\n', cur_joint_state.position[start_i:end_i]
            print ' '


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
