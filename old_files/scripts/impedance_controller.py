# This file runs an impedance controller to stabilize a grasped object
import rospy

import numpy as np
from in_hand_manipulation.srv import *
import sys

from rospkg import RosPack
rp=RosPack()
rp.list()
path=rp.get_path('allegro_driver')+'/scripts'
sys.path.insert(0,path)
path=rp.get_path('biotac_sensors')+'/scripts'
sys.path.insert(0,path)

from allegro_ros_api import handAPI
from biotac_ros_api import biotacAPI

class impedanceController:
    def __init__(self):
        print "Initializing impedance controller"
        rospy.init_node('impedance_controller_node')
        # create allegro api
        self.biotac=biotacAPI(fingers=[0,1,2,3])
        self.hand=handAPI()
        self.start_impedance=False
        self.loop_rate=rospy.Rate(100)
        self.got_init_joint_angles=False
        self.u=np.zeros(16)
        self.u_cmd=np.zeros(16)
        self.f_gain=np.array([1.0,1.0,1.0,1.0])
        self.initial_vf=np.zeros(3)
        self.vf_gain=0.0
        self.dist=np.array([0.01,0.01,0.01,0.01])
        self.ind_des_pose=np.zeros(3)
        #self.init_joint_angles=
    def init_services_ctrl(self):
        enable_impedance=rospy.Service('in_hand_manipulation/impedance/enable',
                      EnableGraspStability,self.enable_impedance)
        update_fingers=rospy.Service('in_hand_manipulation/impedance/update_fingers',
                                     UpdateGraspFingers,self.update_impedance_fingers)
        disable_impedance=rospy.Service('in_hand_manipulation/impedance/disable',
                                        DisableGraspStability,self.disable_impedance)
        while(not rospy.is_shutdown()):
            # run controller
            self.simple_stable_ctrl()
            self.loop_rate.sleep()

    def update_impedance_fingers(self,req):
        self.fingers=req.fingers
        return True
    def disable_impedance(self,req):

        self.start_impedance=False
        return True
        
    def enable_impedance(self,req):
        self.fingers=req.fingers
        self.got_init_joint_angles=False
        self.start_impedance=True
        
        return True
    

    def simple_stable_ctrl(self):
        des_pdc=70.0
        max_J=100.0
        max_theta_dot=20.0
        if(self.start_impedance==True):
            if(self.got_init_joint_angles==False):
                self.u=np.array(self.hand.joint_state.position)
                u_cmd=self.u.copy()
                self.u_cmd=u_cmd
                self.got_init_joint_angles=True
            else:
                u_cmd=self.u_cmd.copy()
            
            # Get biotac pressure reading:
            pdc_arr=self.biotac.get_pdc()
            if(max(pdc_arr)>150):
                print pdc_arr
            u=np.array(self.hand.joint_state.position)
            #print u_cmd[0]
            #u_cmd=self.u_cmd.
            u_dot=np.array(self.hand.joint_state.velocity)
            # virtual frame:
            vf=self.hand.robot_kdl.min_object_pose(u,self.fingers)[0:3]
            # update control law and send to robot
            for i in self.fingers:
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                finger_pose=self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3]
                err=(vf-finger_pose)*np.array([1.0,1.0,1.0])
                
                K_p=1000*np.array([1.0,0.1,10.0])#*self.f_gain[i]#*float(des_pdc-pdc_arr[i])
                #if(K_p<=0.0):
                #    K_p=0.0
                K_d=10.0

                J_inv=np.linalg.pinv(J)
                for m in range(J_inv.shape[0]):
                    for n in range(J_inv.shape[1]):
                        if(np.abs(J_inv[m,n])>max_J):
                           J_inv[m,n]=0.0
                           
                theta_dot=(np.ravel(J.transpose()*np.matrix(K_p*err).transpose())-K_d*u_dot[i*4:i*4+4])
                if(max(abs(theta_dot))>max_theta_dot):
                    theta_dot=max_theta_dot*theta_dot/max(abs(theta_dot))
                
                
                u_cmd[i*4:i*4+4]+=(theta_dot)/100.0

            if(0 not in self.fingers):
                i=0
                if(u[i*4+1]>0.0):
                    theta_dot=[0.0,-0.2,-0.2,-0.2]
                    u_cmd[i*4:i*4+4]+=(theta_dot)


            #print u_cmd
            self.hand.publish_joints_ctrl(u_cmd)

            #self.u_cmd=u_cmd.copy()
        else:
            # TODO: Switch to grav comp
            return True
        
            
if __name__=='__main__':
    allegro_imp=impedanceController()
    allegro_imp.init_services_ctrl()
