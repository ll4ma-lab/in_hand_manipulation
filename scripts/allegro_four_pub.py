#!/usr/bin/env python
import rospy
from std_msgs.msg import Char
from sensor_msgs.msg import JointState
import numpy as np
from object_list import *
import sys
_DISPLAY_RATE = 50

class positionPublisher:
    ''' This class takes in joint position commands, delta_t and publishes the data to the robot'''
    def __init__(self, node_name='allegro_joint_publisher', listen_prefix='/lbr4', listen=False, publish_prefix='/lbr4'):
        rospy.init_node(node_name)

       
        self.allegro_joint_cmd_pub = rospy.Publisher(publish_prefix+'/allegro_hand_right/joint_cmd',
                                                     JointState, queue_size=100)
       
        self.allegro_ctrl_type_pub = rospy.Publisher(publish_prefix+'/allegro_hand_right/control_type', Char,
                                             queue_size=5)


    def send_joint_command(self):
        pass

    def joint_state_cb(self, cur_joint_state):
        if cur_joint_state.header.seq % _DISPLAY_RATE == 0:
            print cur_joint_state.name

    def read_file(self,traj_no):
        
        self.names_index=np.array([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15])
        file_name="ral_linear.csv"
        folder_to_write="ral_linear/"
        '''
        file_name="new_init_1.csv"
        folder_to_write="new/"
        '''
        obj=ObjectList(file_name,folder_to_write)
        f_name=obj.arr[traj_no].f_name
        f=open(f_name,'r')
        self.pos_array=np.loadtxt(f)
        self.delta_t=60.0
    def run_allegro_pub(self):

        run_rate = rospy.Rate(self.delta_t)
        self.allegro_ctrl_type_pub.publish(Char(data='p'))

        run_rate.sleep()

        jc = JointState()
        jc.name = ['index_joint_0','index_joint_1','index_joint_2', 'index_joint_3',
                   'middle_joint_0','middle_joint_1','middle_joint_2', 'middle_joint_3',
                   'ring_joint_0','ring_joint_1','ring_joint_2', 'ring_joint_3',
                   'thumb_joint_0','thumb_joint_1','thumb_joint_2', 'thumb_joint_3']

        jc.position=[ 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0]
        # initial grasp
        jc.position=[ 0.27, 0.87, 0.8, 1.6,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.9, -0.0310, 1.20816, 0.0]
        self.allegro_joint_cmd_pub.publish(jc)
        run_rate.sleep()
        for k in xrange(len(self.pos_array[:,0])):
            for i in xrange(len(self.pos_array[0,:])):
                jc.position[self.names_index[i]]=self.pos_array[k,i]         

            self.allegro_joint_cmd_pub.publish(jc)
            run_rate.sleep()


def test_allegro_traj(listen=False, publish_prefix='', listen_prefix=''):
    jt = positionPublisher(listen=listen, listen_prefix='/allegro_hand_right', publish_prefix='')
    tj=int(sys.argv[1])
    jt.read_file(tj)
    jt.run_allegro_pub()

if __name__=='__main__':

    test_allegro_traj()
        
