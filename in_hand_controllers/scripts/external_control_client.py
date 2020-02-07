import rospy
from in_hand_controllers.srv import *
from hand_services.srv import *
import numpy as np
from geometry_msgs.msg import Pose
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
                return resp.slip_force,resp.last_jcmd,resp.biotac_tare_msg
            else:
                return np.zeros(3),last_jcmd,None
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return np.zeros(3),last_jcmd,None
    def update_impedance_grasp(self,last_jcmd,mu_,mu_slip_,f_idx=[0,1,2,3],des_poses=None):
        # enable impedancce grasp controller

        if des_poses==None:
            des_poses=[Pose() for i in range(len(f_idx))]
        rospy.wait_for_service('/tactile_impedance_control/update_params')
        try:
            req=rospy.ServiceProxy('/tactile_impedance_control/update_params',TactileImpedanceGrasp)
            resp=req(f_idx,last_jcmd,mu_,mu_slip_,des_poses)
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return False

    def toggle_external_controller(self,srv_name=None,flag=True):
        rospy.wait_for_service(srv_name)
        try:
            req=rospy.ServiceProxy(srv_name,ToggleController)
            resp=req(flag)
            return resp.status
        except rospy.ServiceException, e:
            print "Service call failes: %s"%e
        return False
            
    def toggle_impedance_grasp(self,flag=True):
        return self.toggle_external_controller("/tactile_impedance_control/enable",flag)
        
