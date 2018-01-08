import rospy
from in_hand_manipulation_msgs.srv import FingerGaitServo
from in_grasp_manipulation.srv import InGraspManipulation,InGraspSim
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import numpy as np
import tf
class rollingClient:
    def __init__(self,weights=[1.0,0.0,0.4]):
        # Initialize variables:
        self.weights=weights
        
    def call_server(self,js,obj_pose,des_obj_pose,T=10,fingers=[0,1,2]):
        #j0=JointState()
        #j0.position=j_arr

        rospy.wait_for_service('/in_hand/primitive/in_grasp_rolling')
        get_in_grasp_plan=rospy.ServiceProxy('/in_hand/primitive/in_grasp_rolling',InGraspManipulation)
        
        j_traj=get_in_grasp_plan(js,obj_pose,des_obj_pose,self.weights,fingers,3,T,True,JointState(),JointTrajectory())
        return j_traj.joint_traj

class fgClient:
    def call_server(self,finger,init_js,des_js,vel_thresh):
        rospy.wait_for_service('/in_hand/finger_gait')
        get_fg=rospy.ServiceProxy('/in_hand/finger_gait',FingerGaitServo)
        
        resp=get_fg(vel_thresh,init_js,des_js,finger,0.0,False)
        return True
        
