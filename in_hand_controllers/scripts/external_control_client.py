import rospy
from in_hand_controllers.srv import *
import numpy as np

class externalHandControl:
    def __init__(self,init_node=False,node_name='external_hand_control'):
        if(init_node):
            rospy.init_node(node_name)

    def estimate_friction(self,last_jcmd,force_mag,force_pt,force_direction,f_idx=0):
        rospy.wait_for_service('/tactile_friction_estimation/get_estimate')
        try:
            req=rospy.ServiceProxy('/tactile_friction_estimation/get_estimate',TactileFrictionEstimation)
            resp=req(f_idx,last_jcmd,force_mag,force_pt,force_direction)
            if(resp.success):
                return resp.slip_force,resp.last_jcmd
            else:
                return np.zeros(3),last_jcmd
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return np.zeros(3),last_jcmd
    
