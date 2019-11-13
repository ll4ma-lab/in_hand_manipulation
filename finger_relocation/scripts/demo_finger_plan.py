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
    j_config = [0.5 for i in range(len(regrasp_client.j_names))]
    print regrasp_client.update_sim_state(j_config)

    # update object pose:
    o_pose = Pose()
    o_pose.orientation.w = 1.0
    o_pose.position.x = 0.4
    print regrasp_client.update_obj(o_pose)

    # get finger gaiting plan:
    
    
    
