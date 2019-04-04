import rospy
from gait_client import *
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState

from trajectory_smoothing.srv import GetSmoothTraj


import numpy as np
import json

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

path_hand_services=rp.get_path('hand_services')+'/scripts'
sys.path.insert(0,path_hand_services)

from hand_client import handClient
if __name__=='__main__':

    obj_name='mustard_0'
    data_file='../../in_hand_manipulation_data/data/grasps/'+obj_name+'.json'
    
    prev_data = json.load(open(data_file))

    
    # load object mesh
    fg_client=fgClient()
    viz_client=VizInHand(t_name='in_hand/jtraj')
    rrt_viz_client=VizInHand(t_name='in_hand/rrt/jtraj')
    smooth_viz_client=VizInHand(t_name='in_hand/smooth/jtraj')
    hand_srv=handClient()
    allegro_drv=handAPI(init_node=False)

    #raw_input("initial grasp?")
    #print allegro_drv.get_joint_state()

    # move to initial grasp:
    init_grasp=np.array([0.18515369800878298, 0.680156441664917, 0.9638185855530761, 1.4512640354129331, 0.129528500647815, 0.8766460803681153, 0.6737415294890076, 1.7115512938381905, -0.012829824351818855, 0.7482599613406677, 1.0364916317102963, 1.2926487412003786, 1.6792131064308655, 0.07232154412018432, 0.019771989583282478, 1.6049583011069821])
    
    hand_srv.send_pos_cmd(init_grasp)
    raw_input("placed object?")
    
    
    # increase stiffness on other fingers:
    _,stiff_js=hand_srv.increase_stiffness(f_idx=[1,2,3],q_des=allegro_drv.get_joint_state().position)
    

    
    int_marker=intMarker(init_node=False)
    cpt_marker=intMarker(init_node=False)
    mesh_f=prev_data['obj_name']
    int_marker.create_marker(use_mesh=True, mesh=mesh_f, m_name='obj_marker', s_name='obj_marker')



    

    # initialize object at a pose

    # use interactive marker to move object?
    # get object pose from interactive marker:

    #raw_input('moved object?')
    obj_pose=Pose()
    #obj_pose.position,obj_pose.orientation=int_marker.get_marker_pose()
    pose=prev_data['obj_pose']
    obj_pose.position.x= pose[0]#0.103944540024
    obj_pose.position.y=pose[1]#-0.0620919726789
    obj_pose.position.z=pose[2]#-0.0313244313002
    obj_pose.orientation.x=pose[3]#-0.64
    obj_pose.orientation.y=pose[4]#-0.326
    obj_pose.orientation.z=pose[5]#-0.31
    obj_pose.orientation.w=pose[6]#0.61


    # kill server:
    int_marker.clear_marker()
    #obj_pose.orientation.w=1.0
    mesh="/home/bala/object_models/mustard_bottle/google_16k/nontextured.binvox"
    fg_client.update_obj(obj_pose,mesh)
    cpt_marker.create_marker(use_mesh=False, m_name='des_contact_pt', s_name='cpt_marker')
    # give a desired contact point for index finger
    
    # use interactive marker to get desired contact point?
    raw_input("got des_contact pt?")
    pt,_=cpt_marker.get_marker_pose()
    #print pt
    
    des_contact_pt=Point()
    des_contact_pt.x=pt.x
    des_contact_pt.y=pt.y
    des_contact_pt.z=pt.z
    '''

    des_contact_pt.x=0.066#pt.x
    des_contact_pt.y=0.086#pt.y
    des_contact_pt.z=-0.0237#pt.z
    '''
    cpt_marker.clear_marker()
    finger=0

    viz_client.viz_cpts([des_contact_pt])

    # read state from robot:
    init_js=allegro_drv.get_joint_state()
    #print js
    init_js.position=stiff_js
    # generate trajectory
    jtraj,cpt,rrt_traj=fg_client.call(des_contact_pt,finger,init_js,T=10,slide=False)

    rrt_viz_client.viz_jtraj(rrt_traj)
    # visualize trajectory
    viz_client.viz_jtraj(jtraj)

    # smoothen trajectory:
    
    max_acc=np.ones(16)*0.25
    max_vel=np.ones(16)*0.4
    # call service for smoothing:
    rospy.wait_for_service('allegro/get_smooth_trajectory')
    traj_call=rospy.ServiceProxy('allegro/get_smooth_trajectory',GetSmoothTraj)
    resp=traj_call(jtraj,max_acc,max_vel,0.2,0.001)
    #print resp.smooth_traj
    smooth_traj=resp.smooth_traj
    
    smooth_viz_client.viz_jtraj(smooth_traj)

    #
    raw_input("send smooth traj to robot?")
    
    hand_srv.send_jtraj(smooth_traj)

    # increase stiffness on finger:

    
