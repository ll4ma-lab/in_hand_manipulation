import rospy
#from gait_client import *
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState

#from trajectory_smoothing.srv import GetSmoothTraj


import numpy as np
import json

#import sys

#from rospkg import RosPack
#rp=RosPack()
#path_in_hand=rp.get_path('in_hand_manipulation')
#sys.path.insert(0,path_in_hand+'/scripts')
#from viz_util import *
#from interactive_marker import intMarker
#path_allegro=rp.get_path('allegro_driver')+'/scripts'
#sys.path.insert(0,path_allegro)
#from allegro_ros_api import handAPI

#path_hand_services=rp.get_path('hand_services')+'/scripts'
#sys.path.insert(0,path_hand_services)

#from in_hand_manipulation_msgs.srv import UpdateRobotState
from finger_client import regraspClient

#from hand_client import handClient

if __name__=='__main__':
    obj_mesh = "package://ll4ma_scene_description/data/meshes/soft_scrub.stl"
    regrasp_client = regraspClient(obj_mesh,init_node = True)
    
    # move hand to initial grasp:
    j_config =[-0.7162952000000002, 1.3607459999999998, 1.5310522, -1.4783562,
               0.23895700000000009, 0.5382774000000001, 0.042033399999999776,
               -0.3, 0.7413025999999998, 0.8133260000000002,
               0.2216734999999999,
               -0.2, 0.6,1.0, 0,
               0,0,0, 0,
               1.61, 0.2, 0.4,-0.1]

    _,js_init= regrasp_client.update_sim_state(j_config)

    # update object pose:
    o_pose = Pose()
    o_pose.position.x = -0.5
    o_pose.position.y = -0.3
    o_pose.position.z = 0.5

    o_pose.orientation.x = -0.682
    o_pose.orientation.y = 0.0
    o_pose.orientation.z = 0.0
    o_pose.orientation.w = 0.732
    print regrasp_client.update_obj(o_pose)

    # get finger gaiting plan:
    des_cpt = Point() # -0.532, -0.318, 0.424
    des_cpt.x = -0.532
    des_cpt.y = -0.318
    des_cpt.z = 0.5
    raw_input("optimize?")
    jtraj = regrasp_client.plan_finger_gait(1,des_cpt,js_init)

    regrasp_client.send_traj(jtraj)
    
