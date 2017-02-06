#!/usr/bin/env python
import rospy
from std_msgs.msg import Char
from sensor_msgs.msg import JointState
import numpy as np
_DISPLAY_RATE = 50
from object_list import *
import sys
class positionPublisher:
    ''' This class takes in joint position commands, delta_t and publishes the data to the robot'''
    def __init__(self, node_name='joint_tester', listen_prefix='/lbr4', listen=False, publish_prefix='/lbr4'):
        rospy.init_node(node_name)

        self.joint_cmd_pub = rospy.Publisher(publish_prefix+'/lbr4/joint_cmd', JointState,
                                             queue_size=5)

        self.ctrl_type_pub = rospy.Publisher(publish_prefix+'/lbr4/control_type', Char,
                                             queue_size=5)
        if listen:
            rospy.Subscriber(listen_prefix+'/joint_states', JointState,
                             self.joint_state_cb)

        self.allegro_joint_cmd_pub = rospy.Publisher(publish_prefix+'/allegro_hand_right/joint_cmd',
                                                     JointState, queue_size=100)
        self.allegro_torque_cmd_pub = rospy.Publisher('/allegro_hand_right/torque_cmd',
                                                      JointState, queue_size=100)

        self.allegro_ctrl_type_pub = rospy.Publisher(publish_prefix+'/allegro_hand_right/control_type', Char,
                                             queue_size=5)


    def send_joint_command(self):
        pass

    def joint_state_cb(self, cur_joint_state):
        if cur_joint_state.header.seq % _DISPLAY_RATE == 0:
            print cur_joint_state.name

    def run_allegro_pub(self):
        tj=int(sys.argv[1])
        run_rate = rospy.Rate(60.0)
        self.allegro_ctrl_type_pub.publish(Char(data='p'))
        file_name="ral_rerun.csv"
        folder_to_write="ral_centroid/"
        '''
        file_name="objects_relaxed.csv"
        folder_to_write="relaxed-full/"
        '''
        '''
        file_name="new_init_1.csv"
        folder_to_write="junk/"
        '''
        obj_list=ObjectList(file_name,folder_to_write)
        
        run_rate.sleep()

        jc = JointState()
        jc.name = ['index_joint_0','index_joint_1','index_joint_2', 'index_joint_3',
                   'middle_joint_0','middle_joint_1','middle_joint_2', 'middle_joint_3',
                   'ring_joint_0','ring_joint_1','ring_joint_2', 'ring_joint_3',
                   'thumb_joint_0','thumb_joint_1','thumb_joint_2', 'thumb_joint_3']
        
        # initial grasp
        i=tj
        print obj_list.arr[i].name
        print obj_list.arr[i].grasp

        #jc.position=[0.1, 1.0, 0.7, 1.6, 0.0, 1.0, 0.8, 1.6, 0.0, 0.0, 0.0, 0.0, 1.4, 0.25, 0.7, 1.2]
        #jc.position[0:12]=[0.0]*12
        #print jc.position
        jc.position=obj_list.arr[i].grasp
        #jc.position[12]=0.8
        for i in range(100):
            self.allegro_joint_cmd_pub.publish(jc)
            run_rate.sleep()


def test_allegro_traj(listen=False, publish_prefix='', listen_prefix=''):
    jt = positionPublisher(listen=listen, listen_prefix='', publish_prefix='')
    
    jt.run_allegro_pub()

if __name__=='__main__':

    test_allegro_traj()
        
