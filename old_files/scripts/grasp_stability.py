# This file has functions to perform grasp stability with the allegro hand and biotacs.
import numpy as np
import rospy
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
        print "grasp stability class initialized"
        rospy.init_node('grasp_stability_node')
        # create allegro api
        self.biotac=biotacAPI(fingers=[0,1,2,3])
        self.hand=handAPI()

        
        
    def grasp_preshape(self,j0):
        # move to grasp preshape
        print "going to preshape"
        self.hand.publish_joints(j0)

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
        
    def grasp_object(self,pdc=30):
        # grasp object until a pressure is reached on the biotacs
        print "grasping object"
        print self.biotac.get_pdc()
        reached_pressure=False
        j_delta=np.zeros(16)
        j_delta[1]=0.01
        j_delta[5]=0.01
        j_delta[9]=0.01
        j_delta[14]=0.01
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
            
        
        
    def impedance_mode(self,fingers=[0,3]):
        # Enable impedance stability mode with fingers
        print "impedance mode"

        #while(not rospy.is_shutdown()):
            # impedance controller
            # 

if __name__=='__main__':
    g_st=graspStability()
    pre_shape=np.array([0.2,0.9,0.7,0.1,
                       0.2,0.9,0.7,0.1,
                       0.2,0.9,0.7,0.1,
                        1.53,-0.06,0.3,0.2])

    raw_input("go to preshape?")
    g_st.grasp_preshape(pre_shape)


    raw_input("Grasp object?")

    g_st.grasp_object()
    
    '''
    raw_input("Go to impedance mode?")

    g_st.impedance_mode()
    '''
