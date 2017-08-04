import rospy
from in_hand_manipulation.srv import PlanFingerGait
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
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


class fingerGaitClient:
    def __init__(self):
        print "Initialized finger gait client"
    def call_server(self,init_joints,obj_pose,desired_finger_pt,finger):
        rospy.wait_for_service('/in_hand/primitive/finger_gaiting')
        finger_gait=rospy.ServiceProxy('/in_hand/primitive/finger_gaiting',PlanFingerGait)
        j_traj=finger_gait(init_joints,obj_pose,desired_finger_pt,finger)
        return j_traj.joint_traj



if __name__=='__main__':
    file_=path_in_grasp+"/data/trajectories/exp_2_goal_poses.csv"
    obj_list=ObjectList(file_,' ')
    obj_=obj_list.arr[7]

    j0=obj_.grasp

    
    f_g=fingerGaitClient()
    init_joints=j0
    #init_joints[0:4]=[0.10242784913979935, 0.9247851607447277, 1.3499999999999996, 1.349999999999997]
    js=JointState()
    js.position=init_joints
    obj_pose=Pose()
    obj_pose.position.x=0.0
    finger_pt=Pose()
    finger_pt.position.x=0.06
    finger_pt.position.y=0.08
    finger_pt.position.z=0.0
    
    j_traj=f_g.call_server(js,obj_pose,finger_pt,0)
    print j_traj.points[-1].positions
    # Publish joint traj:
    t_name='planner/finger_gait_traj'
    traj_server=trajectoryServer(100,robot_name='allegro',topic_name=t_name,init_node=True)
    time.sleep(1.0)
    traj_server.viz_joint_traj(j_traj,'allegro_mount')
