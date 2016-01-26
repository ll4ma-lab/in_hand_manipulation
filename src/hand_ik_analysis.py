#!/usr/bin/env python

#import rospy
import roslib
import rospy
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

import numpy as np
import matplotlib.pyplot as plotter
import scipy.optimize as opt
from math import sin, cos, pi
import time

class HandSimpleModel:
    def __init__(self, urdf_file_name):
        '''
        Simple model of the hand hand kinematics and controls
        Assume following state and action vectors
        urdf_file_name - model file to load
        '''
        # rospy.init_node('hand_model_node')

        # Load KDL trees for each hand finger
        urdf_file = file(urdf_file_name, 'r')
        self.robot = Robot.from_xml_string(urdf_file.read())
        urdf_file.close()
        self.tree = kdl_tree_from_urdf_model(self.robot)
        print '\nNum segements:',self.tree.getNrOfSegments(),'\n'

        # TODO: Get list of base link for all fingers
        base_link = self.robot.get_root()
        # base_link = 'palm_link'
        # TODO: Get list of each finger end-effector
        self.finger_chains = []
        ee_list = ['link_3.0_tip', 'link_7.0_tip', 'link_11.0_tip', 'link_15.0_tip']

        for i, ee_name in enumerate(ee_list):
            fc = KDLKinematics(self.robot, base_link, ee_name)
            self.finger_chains.append(fc)
            print 'Link', i, 'num joints:',fc.chain.getNrOfJoints()
            # TODO: Set joint limits here

    def FK(self, Q):
        '''
        Method to convert joint positions to task coordinates
        '''
        X = np.zeros(6)
        Q_f1, Q_f2 = self.Q_to_finger_joints(Q)
        self.f1.forward(Q_f1)
        if all_links:
            # TODO: Append proximal link location
            pass
        return X

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
        # preshape = pi*0.5
        # q0 = 0
        # q3 = 0.4
        # Q = np.array([q0,0,0,q3, preshape])
        # self.draw_robot(Q,False)
        # q0 = 1.3
        # q1 = 0.3
        # q2 = 0.4
        # q3 = 0
        # Q = np.array([q0,q1,q2,q3, preshape])
        # self.draw_robot(Q,True)
        pass
        # rospy.spin()

def main():
    urdf_file_name = roslib.packages.get_pkg_dir('allegro_hand_description') + '/allegro_hand_description_right.urdf'
    model = HandSimpleModel(urdf_file_name)

    model.run()
    return model

if __name__ == '__main__':
    main()
