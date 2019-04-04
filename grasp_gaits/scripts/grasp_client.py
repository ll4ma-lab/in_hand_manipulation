import rospy
from in_hand_manipulation_msgs.srv import GraspObj, UpdateObj
from geometry_msgs.msg import Pose,Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from in_hand_manipulation_msgs.srv import PrecisionGrasp
class graspClient:
    def call(self,des_contact_pts,init_js,T=10):
        rospy.wait_for_service('in_hand/grasp')
        fg=rospy.ServiceProxy('in_hand/grasp', GraspObj)
        resp=fg(des_contact_pts,init_js,T,Pose(),[0,1,2,3],JointTrajectory())
        return resp.joint_traj
    def update_obj(self,obj_pose,mesh):
        rospy.wait_for_service('in_hand/grasp/update_obj')
        obj=rospy.ServiceProxy('in_hand/grasp/update_obj',UpdateObj)
        obj(obj_pose,mesh)
        return True
    def send_robot(self,jtraj):
        rospy.wait_for_service('grasp/control')
        resp=rospy.ServiceProxy('grasp/control',PrecisionGrasp)
        resp(0.9,jtraj,True)

# service call

# finger gait all fingers

# in-grasp manipulation 

# return generated sequence
