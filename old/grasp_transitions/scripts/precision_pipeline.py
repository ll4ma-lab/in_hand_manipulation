import rospy
import sys
import numpy as np
from rospkg import RosPack
rp=RosPack()
rp.list()
path_allegro=rp.get_path('allegro_driver')+'/scripts'
sys.path.insert(0,path_allegro)
from allegro_ros_api import handAPI


class precisionGrasp:
    def __init__(self, publish_prefix='/allegro_hand_right',listen_prefix='/allegro_hand_right', init_node=True, node_name='precision_grasp'):
        if(init_node):
            rospy.init_node(node_name)

        self.allegro_drv=handAPI(init_node=False,publish_prefix=publish_prefix,listen_prefix=listen_prefix)



if __name__=='__main__':
    p_grasp=precisionGrasp()
    '''
    [-0.06863077273130495, 1.140887736710028, 0.32654539240656755, 0.4414865585173827, 0.08664525212940671, 1.2893973473577942, 0.15923042277736818, 0.29596471518442385, 0.10712024578676148, 1.4459915048573913, -0.09508130101827397, 0.6051986322669617, 1.6300028212458073, 0.1363827903700196, 0.5883265344892274, -0.1847143204624878]
    '''
    # preshape
    j0=np.array([0.2,1.0,0.3,0.45,
                 0.0,1.0,0.3,0.45,
                 -0.2,1.0,0.3,0.45,
                 1.62,0.13,0.4,0.0])
    
    raw_input('Move to initial grasp?')
    p_grasp.allegro_drv.publish_joints(j0)

    # tactile grasp controller
    raw_input('Grasp object?')
    j0=np.array([0.2,1.0,0.5,0.7,
                 0.0,1.0,0.5,0.7,
                 -0.2,1.0,0.5,0.7,
                 1.62,0.13,0.5,0.3])
    
    p_grasp.allegro_drv.publish_joints(j0)

    # release object?
    j0=np.array([0.2,1.0,0.3,0.45,
                 0.0,1.0,0.3,0.45,
                 -0.2,1.0,0.3,0.45,
                 1.62,0.13,0.4,0.0])
    
    raw_input('Release object?')
    p_grasp.allegro_drv.publish_joints(j0)
    
    # lift object
