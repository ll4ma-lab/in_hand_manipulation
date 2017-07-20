# This file has functions to perform grasp stability with the allegro hand and biotacs.
import numpy as np
import rospy
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

class graspStability:
    def __init__(self):
        print "Initializing grasp stability services"
        rospy.init_node('grasp_stability_services_node')
        # create allegro api
        self.biotac=biotacAPI(fingers=[0,1,2,3])
        self.hand=handAPI()
        
        
        
    def grasp_preshape(self,req):
        j0=req.hand_joint_angles
        self.hand.publish_joints(j0)
        # TODO: wait for joints to reach preshape
        return True
    
    def grasp_object_pos(self,j0):
        print "grasping object by joint command until contact is detected"
        reached_pressure=False
        j_delta=np.zeros(16)
        
        i=0
        j_pos=self.hand.joint_state.position
        new_pos=j_pos+j_delta
        while(reached_pressure==False and i<100):
            new_pos=new_pos+j_delta
            self.hand.publish_joints(new_pos)
            print self.biotac.get_pdc()
            i+=1
            if((self.biotac.get_pdc()>pdc).all()):
                reached_pressure=True
        
    def grasp_object(self,req):
        pdc=req.pdc
        # grasp object until a pressure is reached on the biotacs
        print "grasping object"
        print self.biotac.get_pdc()
        reached_pressure=False
        j_delta=np.zeros(16)
        j_delta[1]=0.05
        j_delta[5]=0.05
        j_delta[9]=0.05
        j_delta[14]=0.05
        i=0
        j_pos=self.hand.joint_state.position
        new_pos=j_pos+j_delta
        while(reached_pressure==False):
            new_pos=new_pos+j_delta
            self.hand.publish_joints(new_pos)
            #print self.biotac.get_pdc()
            for i in range(len(self.biotac.get_pdc())):
                if(self.biotac.get_pdc()[i]>pdc):
                    # stop moving the finger i
                    j_delta[i*4:i*4+4]=np.zeros(4)
            if((self.biotac.get_pdc()>pdc).all()):
                reached_pressure=True
        print "grasped object"
        print self.biotac.get_pdc()
        return reached_pressure
        
        
        
    def init_services(self):
        send_preshape=rospy.Service('in_hand_manipulation/send_preshape',SendPreshapeArr,self.grasp_preshape)
        grasp_object=rospy.Service('in_hand_manipulation/grasp_object',BiotacGraspObject,self.grasp_object)
        rospy.spin()

if __name__=='__main__':
    g_st=graspStability()
    g_st.init_services()
