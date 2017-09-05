import rospy
from in_hand_manipulation.srv import GetGrasp
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import sys
import time
# import required allegro model files
from rospkg import RosPack

rp=RosPack()
rp.list()

path_=rp.get_path('optimization_pkg')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *

path_in_grasp=rp.get_path('in_grasp_manipulation')
sys.path.insert(0,path_in_grasp+'/python')
from object_list import *


class graspClient:
    def __init__(self):
        print "Initialized finger gait client"
    def call_server(self,obj_pose):
        rospy.wait_for_service('/in_hand/gen_grasp')
        finger_gait=rospy.ServiceProxy('/in_hand/gen_grasp',GetGrasp)
        res=finger_gait(obj_pose,[0,1,2,3])
        return res.joint_state,res.n_obj_pose,res.cpts,res.grasp_score



if __name__=='__main__':
    g=graspClient()
    obj_pose=PoseStamped()
    obj_pose.pose.position.x=0.07
    obj_pose.pose.position.z=-0.02
    obj_pose.pose.position.y=0.0
    obj_pose.pose.orientation.z=0.174
    obj_pose.pose.orientation.w=0.985

    obj_pose.pose.orientation.x=0.062
    obj_pose.pose.orientation.y=0.704
    obj_pose.pose.orientation.z=0.062
    obj_pose.pose.orientation.w=0.704

    
    js,o_pose,cpts,g_score=g.call_server(obj_pose)
    
    
    # Publish joint traj:

    t_name='planner/grasp'
    traj_server=trajectoryServer(100,robot_name='allegro',topic_name=t_name,init_node=True)
    time.sleep(1.0)
    j_traj=traj_server.get_jtraj(js)
    traj_server.viz_joint_traj(j_traj,'allegro_mount')

