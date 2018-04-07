import rospy
from gait_client import *
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
import numpy as np

import sys

from rospkg import RosPack
rp=RosPack()
path_in_hand=rp.get_path('in_hand_manipulation')
sys.path.insert(0,path_in_hand+'/scripts')
from viz_util import *
from interactive_marker import intMarker
path_allegro=rp.get_path('allegro_driver')+'/scripts'
sys.path.insert(0,path_allegro)
from allegro_ros_api import handAPI

if __name__=='__main__':
    # load object mesh
    fg_client=fgClient()
    viz_client=VizInHand(t_name='in_hand/jtraj')
    allegro_drv=handAPI(init_node=False,publish_prefix='',listen_prefix='')

    int_marker=intMarker(init_node=False)
    cpt_marker=intMarker(init_node=False)

    int_marker.create_marker(use_mesh=True, mesh='file:///home/bala/object_models/mustard_bottle/google_16k/nontextured.stl', m_name='obj_marker', s_name='obj_marker')

    

    # initialize object at a pose

    # use interactive marker to move object?
    # get object pose from interactive marker:

    raw_input('moved object?')
    obj_pose=Pose()
    obj_pose.position,obj_pose.orientation=int_marker.get_marker_pose()
    '''
    obj_pose.position.x= 0.103944540024
    obj_pose.position.y=-0.0620919726789
    obj_pose.position.z=-0.0313244313002
    obj_pose.orientation.x=-0.64
    obj_pose.orientation.y=-0.326
    obj_pose.orientation.z=-0.31
    obj_pose.orientation.w=0.61
    '''

    # kill server:
    int_marker.clear_marker()
    #obj_pose.orientation.w=1.0
    mesh="/home/bala/object_models/mustard_bottle/google_16k/nontextured.binvox"
    #fg_client.update_obj(obj_pose,mesh)
    cpt_marker.create_marker(use_mesh=False, m_name='des_contact_pt', s_name='cpt_marker')
    # give a desired contact point for index finger
    
    # use interactive marker to get desired contact point?
    raw_input("got des_contact pt?")
    pt,_=cpt_marker.get_marker_pose()
    des_contact_pt=Point()

    des_contact_pt.x=pt.x
    des_contact_pt.y=pt.y
    des_contact_pt.z=pt.z
    '''

    des_contact_pt.x=0.079#pt.x
    des_contact_pt.y=0.018#pt.y
    des_contact_pt.z=0.011#pt.z
    '''
    cpt_marker.clear_marker()
    finger=1

    viz_client.viz_cpts([des_contact_pt])

    # read state from robot:
    init_js=allegro_drv.get_joint_state()

    
    
    # generate trajectory
    jtraj,cpt=fg_client.call(des_contact_pt,finger,init_js,T=1,slide=False)

    # visualize trajectory
    viz_client.viz_jtraj(jtraj)
