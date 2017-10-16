# This file calls different clients to perform specific tasks
import rospy
import numpy as np
from in_hand_manipulation.srv import *
def grasp_object(j0,pdc=10):
    # Move hand to preshape:
    rospy.wait_for_service('in_hand_manipulation/send_preshape')
    try:
        send_preshape=rospy.ServiceProxy('in_hand_manipulation/send_preshape',SendPreshapeArr)
        resp1=send_preshape(j0)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

    raw_input('Grasp object?')
    # Grasp object:
    rospy.wait_for_service('in_hand_manipulation/send_preshape')
    try:
        grasp_call=rospy.ServiceProxy('in_hand_manipulation/grasp_object',BiotacGraspObject)
        resp1=grasp_call(pdc)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

def pressure_ctrl(pdc=10):
    
    rospy.wait_for_service('in_hand_manipulation/send_preshape')
    try:
        grasp_call=rospy.ServiceProxy('in_hand_manipulation/grasp_object',BiotacGraspObject)
        resp1=grasp_call(pdc)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

def impedance_mode(fingers=[0,1,2,3]):
    # Once fingers are in contact with the object, switch to impedance mode:
    rospy.wait_for_service('in_hand_manipulation/impedance/enable')
    try:
        grasp_call=rospy.ServiceProxy('in_hand_manipulation/impedance/enable',EnableGraspStability)
        gain=0.1
        resp1=grasp_call(fingers,gain)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

def stop_impedance_mode():
    # Once fingers are in contact with the object, switch to impedance mode:
    rospy.wait_for_service('in_hand_manipulation/impedance/disable')
    try:
        grasp_call=rospy.ServiceProxy('in_hand_manipulation/impedance/disable',DisableGraspStability)
        resp1=grasp_call()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

def update_fingers_imp(fingers=[1,2,3]):
    rospy.wait_for_service('in_hand_manipulation/impedance/update_fingers')
    try:
        srv_call=rospy.ServiceProxy('in_hand_manipulation/impedance/update_fingers',UpdateGraspFingers)
        resp1=srv_call(fingers)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    
    
                           
if __name__=='__main__':

    #impedance_mode()
    #raw_input("?")
    #while(i<10):
        # Compute preshape:
    pre_shape=np.array([0.2,0.7,0.7,0.1,
                        0.2,0.7,0.7,0.1,
                        0.2,0.7,0.7,0.1,
                        1.53,-0.06,0.25,0.0])
    grasp_object(pre_shape,15)
    '''
    raw_input("pressure controller?")
    
    pressure_ctrl(50)
    '''
    raw_input("impedance mode?")
    # impedance mode
    impedance_mode()

    raw_input("update_fingers?")
    update_fingers_imp()

    # finger sliding
    

        
    raw_input("stop imp mode?")
    stop_impedance_mode()

    raw_input("go back to preshape?")
    grasp_object(pre_shape,20)
        
    

