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
        print "Initializing pressure controller"
        rospy.init_node('pressure_controller_node')
        # create allegro api
        self.biotac=biotacAPI(fingers=[0,1,2,3])
        self.hand=handAPI()
        self.start_controller=False
        self.loop_rate=rospy.Rate(100)

    def init_services_ctrl(self):
        enable_controller=rospy.Service('in_hand_manipulation/pressure_controller/enable',
                                       EnableGraspStability,self.enable_pressure)
        update_fingers=rospy.Service('in_hand_manipulation/pressure_controller/update_fingers',
                                     UpdateGraspFingers,self.update_fingers)
        disable_controller=rospy.Service('in_hand_manipulation/pressure_controller/disable',
                                        DisableGraspStability,self.disable_pressure)
        while(not rospy.is_shutdown()):
            # run controller
            self.pressure_ctrl()
            self.loop_rate.sleep()

    def update_fingers(self,req):
        self.fingers=req.fingers
        return True
    def disable_pressure(self,req):
        self.start_controller=False
        return True
        
    def enable_pressure(self,req):
        self.fingers=req.fingers
        self.pdc=req.gain
        self.start_controller=True
        return True
    
    def pressure_ctrl(self):
        des_pdc=self.pdc
        if(self.start_controller==True):
            reached_pressure=False
            j_delta=np.zeros(16)
            j_delta[1]=0.05
            j_delta[5]=0.05
            j_delta[9]=0.05
            j_delta[14]=0.05
            for i in range(len(self.biotac.get_pdc())):
                if(self.biotac.get_pdc()[i]>pdc):
                    # stop moving the finger i
                    j_delta[i*4:i*4+4]=np.zeros(4)
            j_pos=self.hand.joint_state.position
            new_pos=j_pos+j_delta

            new_pos=new_pos+j_delta
            self.hand.publish_joints(new_pos)
            #print self.biotac.get_pdc()
        else:
            # TODO: Switch to grav comp
            return True
            
            
if __name__=='__main__':
    allegro_imp=impedanceController()
    allegro_imp.init_services_ctrl()
