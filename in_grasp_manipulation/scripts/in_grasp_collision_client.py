import rospy
from pcl_mesh_tools.srv import PreprocessCloud,update_env
from in_grasp_manipulation.srv import InGraspManipulation,InGraspSim
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import numpy as np
import tf
from sensor_msgs.msg import PointCloud2


class inGraspCollisionClient:
    def __init__(self,init_node=False):
        # Initialize variables:
        if(init_node):
            rospy.init_node('in_grasp_client')


    def process_env_cloud(self):
        rospy.wait_for_service("collision_checker/update_env_cloud")
        pre_proc=rospy.ServiceProxy("collision_checker/update_env_cloud",update_env)
        res=pre_proc()


        
if __name__=='__main__':
    in_grasp=inGraspCollisionClient()
    in_grasp.process_env_cloud()
    #in_grasp.call_in_grasp_planner()
    #in_grasp.process_env_cloud()
