#
import rospy
#from in_hand_manipulation.srv import PlanFingerGait
from in_hand_manipulation.srv import GetHandIK
from grasp_gaits.srv import GetEllipse
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np

class ellipseClient:
    def __init__(self):
        print "Yay! initialized ellipse client"
    def call(self):
        print 'TODO'
    def tester(self,f_index=[0]):
        p=PoseStamped()
        p.pose.position.x=0.08
        p.pose.position.y=-0.08
        p.pose.position.z=0.015
        p.pose.orientation.x=-0.706825
        p.pose.orientation.y=0.0
        p.pose.orientation.z=0.0
        p.pose.orientation.w=0.707
        
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)

        rospy.wait_for_service('/regrasp_planner/update_contact_ellipse')
        get_ellipse=rospy.ServiceProxy('/regrasp_planner/update_contact_ellipse',GetEllipse)
        res=get_ellipse(p,f_)
        return res.ellipses

class handIKClient:
    def __init__(self):
        print "Yay! initialized hand ik client"
        
    def call(self,f_index,des_poses):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)
        js=JointState()
        js.position=np.zeros(16)

        d_poses=[]
        for i in range(len(des_poses)):
            p=Pose()
            p.position.x=des_poses[i][0]
            p.position.y=des_poses[i][1]
            p.position.z=des_poses[i][2]
            p.orientation.x=0.0
            p.orientation.y=0.0
            p.orientation.z=0.0
            p.orientation.w=1.0
            d_poses.append(p)

        rospy.wait_for_service('/in_hand/ik')
        get_ik=rospy.ServiceProxy('/in_hand/ik',GetHandIK)
        j_=get_ik(f_,js,d_poses,False)
        
        return j_.joint_state.position

    def tester(self,f_index=[2,1]):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)
        js=JointState()
        js.position=np.zeros(16)

        d_poses=[]
        for i in range(4):
            p=Pose()
            p.position.x=0.03
            p.position.y=0.0
            p.position.z=0.03
            p.orientation.x=0.0
            p.orientation.y=0.0
            p.orientation.z=0.0
            p.orientation.w=1.0
            d_poses.append(p)

        rospy.wait_for_service('/in_hand/ik')
        get_ik=rospy.ServiceProxy('/in_hand/ik',GetHandIK)
        j_=get_ik(f_,js,d_poses,False)
        
        return j_.joint_state.position
        
if __name__=='__main__':

    # Test hand ik service:
    #hand_ik=handIKClient()
    #print hand_ik.tester()
    
    # Test contact ellipse server:
    contact_client=ellipseClient()
    print contact_client.tester()
    
    # Run ros service client:
    # Initialize object and reachable workspace

    # Get initial object pose, contact points and joint angles of initial grasp

    # Compute finger gait contact location

    # Update grasp

    # In-grasp to expand workspace
