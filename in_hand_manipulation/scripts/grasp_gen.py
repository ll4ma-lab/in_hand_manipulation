# this script allows for generating grasp:


# a grasp consists of :
# joint configuration of hand.
# fingertip poses in object frame.
# object pose with respect to the palm.
# using csv file

import rospy

from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from viz_util import *
from interactive_marker import intMarker
import numpy as np
import sys

from rospkg import RosPack
rp=RosPack()
path_allegro=rp.get_path('allegro_driver')+'/scripts'
sys.path.insert(0,path_allegro)
from allegro_ros_api import handAPI
import json

if __name__=='__main__':

    #create data frame:
    # initialize node:
    data_file='../../in_hand_manipulation_data/data/grasps/'+sys.argv[1]+'.json'

    #prev_data = json.load(open(data_file))
    viz_client=VizInHand(t_name='in_hand/jtraj')
    allegro_drv=handAPI(init_node=False,publish_prefix='',listen_prefix='')
    
    int_marker=intMarker(init_node=False)

    obj_src='file:///home/bala/object_models/'
    obj_name='mustard_bottle/google_16k/nontextured.stl'
    obj_mesh=obj_src+obj_name
    # Move object
    int_marker.create_marker(use_mesh=True, mesh=obj_mesh, m_name='obj_marker', s_name='obj_marker')
    
    raw_input('moved object?')
    obj_pose=Pose()
    obj_pose.position,obj_pose.orientation=int_marker.get_marker_pose()
    #build T mat from
    pose=[]
    pose.append(obj_pose.position.x)
    pose.append(obj_pose.position.y)
    pose.append(obj_pose.position.z)
    pose.append(obj_pose.orientation.x)
    pose.append(obj_pose.orientation.y)
    pose.append(obj_pose.orientation.z)
    pose.append(obj_pose.orientation.w)
    o_T_p=np.linalg.inv(allegro_drv.robot_kdl.get_T(pose))
    # move joints
    raw_input("got des_contact pt?")
    des_js=allegro_drv.get_joint_state()
    q=np.array(des_js.position)
    q_state=des_js.position
    cpts=[]
    # get fk of joints and transform to object frame:
    for i in range(4):      
        T=allegro_drv.robot_kdl.end_effector_pose(q[i*4:i*4+4],i)
        o_T=o_T_p*T
        cpts.append([T[0,3],T[1,3],T[2,3]])

    # save current grasp:
    #[obj_name,obj_pose,q,cpts]
    # ask for object reposing/finger changes:
    
    #obj = {u"answer": [42.2], u"abs": 42}

    obj={u"obj_name": obj_name, u"obj_pose":pose, u"js":q_state, u"cpts":cpts}
    #print obj
    with open(data_file, 'w') as outfile:
        json.dump(obj,outfile)
