import rospy
from in_grasp_manipulation.srv import InGraspManipulation
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
class inGraspClient:
    def __init__(self):
        # Initialize variables:
        self.weights=[1.0,1.0,1.0,1.0]
        
    def call_server(self,j0,obj_pose,des_obj_pose):
        rospy.wait_for_service('/in_hand/primitive/in_grasp_manipulation')
        get_in_grasp_plan=rospy.ServiceProxy('/in_hand/primitive/in_grasp_manipulation',InGraspManipulation)
        j_traj=get_in_grasp_plan(j0,obj_pose,des_obj_pose,self.weights)
        return j_traj


if __name__=='__main__':
    # in grasp data:
    j0=JointState()
    j0.position=np.zeros(16)
    obj_pose=Pose()
    obj_pose.position.x=0.0
    obj_pose.position.y=0.0
    obj_pose.position.z=0.0
    obj_pose.orientation.x=0.0
    obj_pose.orientation.y=0.0
    obj_pose.orientation.z=0.0
    obj_pose.orientation.w=1.0

    des_obj_pose=Pose()
    des_obj_pose=obj_pose
    in_grasp=inGraspClient()
    j_traj=in_grasp.call_server(j0,obj_pose,des_obj_pose)
    print j_traj
    
