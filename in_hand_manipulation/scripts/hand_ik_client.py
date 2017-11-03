import rospy
from in_hand_manipulation_msgs.srv import GetHandIK
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import sys
import time

class handIKClient:
    def __init__(self):
        print "Initialized hand IK client"
    def call_server(self,init_joints,desired_poses,f_index,full_pose=False):
        rospy.wait_for_service('/in_hand/ik')
        ik_srv=rospy.ServiceProxy('/in_hand/ik',GetHandIK)
        j_traj=ik_srv(f_index,init_joints,desired_poses,full_pose)
        return j_traj.joint_state,j_traj.joint_traj


'''
def test_ik():
    client=handIKClient()
    joints_0=JointState()
    joints_0.position=np.zeros(16)
    des_pose=Pose()
    des_pose.position.x=0.0
    des_pose.position.y=0.0
    des_pose.position.z=0.0

    
    f_index=[0,1,2,3]
    des_pose_arr=[]
    for i in range(len(f_index)):
        des_pose_arr.append(des_pose)
    final_js=client.call_server(joints_0,des_pose_arr,f_index)
    print final_js
    
if __name__=='__main__':
    test_ik()
'''
