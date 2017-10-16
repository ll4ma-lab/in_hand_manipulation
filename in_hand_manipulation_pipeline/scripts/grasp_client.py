#!/usr/bin/env python
import rospy
from in_hand_manipulation_pipeline.srv import *
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class graspClient:
    def __init__(self):
        print "initialized grasp client"
    def send_preshape(self,q_arr):
        jc=JointState()
        jc.position=q_arr
        rospy.wait_for_service('grasp/send_preshape')
        execute_preshape=rospy.ServiceProxy('grasp/send_preshape',SendPreshape)
        resp1=execute_preshape(jc)
        #print resp1.reached
        return resp1.reached
        
    def grasp_servo_joints(self,vel_thresh,delta_v,joints_to_move):
        rospy.wait_for_service('grasp/grasp_vel_servo')
        vel_servo=rospy.ServiceProxy('grasp/grasp_vel_servo',GraspVelServo)
        thresh=Float64()
        thresh.data=vel_thresh
        del_v=Float64()
        del_v.data=delta_v
        joints=joints_to_move
        resp1=vel_servo(thresh,del_v,joints)
        print resp1.grasped
        return resp1.grasped

    def move_delta_q(self,delta_q,joints_to_move):
        rospy.wait_for_service('grasp/move_delta_q_joints')
        del_q_srv=rospy.ServiceProxy('grasp/move_delta_q_joints',DeltaqJoints)
        del_q=Float64()
        del_q.data=delta_q
        joints=joints_to_move
        resp1=del_q_srv(del_q,joints)
        print resp1.status
        return resp1.status
        



def cmd_zeros():
    grasp=graspClient()
    new_pose=np.zeros(16)
    grasp.send_preshape(new_pose)

def grasp_bottle():
    grasp=graspClient()

    new_pose=np.array([-0.03,0.5,0.8,1.3,
                       0.0,0.154,1.58,0.63,
                       -0.01,0.35,1.0,1.42,
                       1.6,-0.06,0.06,1.35])
    grasp.send_preshape(new_pose)
    raw_input("close hand?")
    joints=[1,2,5,6,9,10,14,15]
    grasp.grasp_servo_joints(0.008,0.008,joints)
    raw_input("remove finger")
    # Remove one finger:
    joints=[10,11]
    grasp.move_delta_q(-0.2,joints)

    raw_input("place finger")
    grasp.grasp_servo_joints(0.008,0.01,joints)
    

if __name__=="__main__":
    #cmd_zeros()
    grasp_bottle()
    
