import rospy
from grasp_gaits.srv import RelocateFingerKin, UpdateObj
from geometry_msgs.msg import Pose,Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
class fgClient:
    def call(self,des_contact_pt,finger,init_js,T=1):
        rospy.wait_for_service('in_hand/finger_gait_nc')
        fg=rospy.ServiceProxy('in_hand/finger_gait_nc',RelocateFingerKin)
        resp=fg(finger,des_contact_pt,init_js,T,Pose(),Point(),JointTrajectory())

        rospy.wait_for_service('in_hand/finger_gait')
        fg=rospy.ServiceProxy('in_hand/finger_gait',RelocateFingerKin)
        resp=fg(finger,resp.next_contact_pt.point,init_js,T,Pose(),Point(),resp.joint_traj)
        return resp.joint_traj,resp.next_contact_pt
    def update_obj(self,obj_pose,mesh):
        rospy.wait_for_service('in_hand/finger_gait/update_obj')
        obj=rospy.ServiceProxy('in_hand/finger_gait/update_obj',UpdateObj)
        obj(obj_pose,mesh)

        rospy.wait_for_service('in_hand/finger_gait_nc/update_obj')
        obj_1=rospy.ServiceProxy('in_hand/finger_gait_nc/update_obj',UpdateObj)
        obj_1(obj_pose,mesh)

        return True

# service call

# finger gait all fingers

# in-grasp manipulation 

# return generated sequence
