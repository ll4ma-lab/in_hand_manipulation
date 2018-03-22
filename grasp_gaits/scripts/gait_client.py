import rospy
from in_hand_manipulation_msgs.srv import RelocateFingerKin, UpdateObj
from geometry_msgs.msg import Pose,Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
class fgClient:
    def call(self,des_contact_pt,finger,init_js,final_js=JointState(),T=1,slide=True,contact_norm=Point()):
        traj=JointTrajectory()
        pt=JointTrajectoryPoint()
        pt.positions=final_js.position
        traj.points.append(pt)
        #traj.points[0].positions=final_js.position
        '''
        rospy.wait_for_service('in_hand/finger_gait_nc')
        fg=rospy.ServiceProxy('in_hand/finger_gait_nc',RelocateFingerKin)
        resp=fg(finger,des_contact_pt,init_js,T,Pose(),Point(),traj)
        '''
        if(slide==False):
            rospy.wait_for_service('in_hand/finger_gait')
            fg=rospy.ServiceProxy('in_hand/finger_gait',RelocateFingerKin)
        else:
            rospy.wait_for_service('in_hand/finger_slide')
            fg=rospy.ServiceProxy('in_hand/finger_slide',RelocateFingerKin)
        resp=fg(finger,des_contact_pt,init_js,T,Pose(),Point(),traj,contact_norm)
        resp=fg(finger,des_contact_pt,init_js,10,Pose(),Point(),resp.joint_traj,contact_norm)
        return resp.joint_traj,resp.next_contact_pt
    def update_obj(self,obj_pose,mesh):
        rospy.wait_for_service('in_hand/finger_gait/update_obj')
        obj=rospy.ServiceProxy('in_hand/finger_gait/update_obj',UpdateObj)
        obj(obj_pose,mesh)

        #rospy.wait_for_service('in_hand/finger_slide/update_obj')
        #obj_1=rospy.ServiceProxy('in_hand/finger_slide/update_obj',UpdateObj)
        #obj_1(obj_pose,mesh)

        '''
        rospy.wait_for_service('in_hand/finger_gait_nc/update_obj')
        obj_1=rospy.ServiceProxy('in_hand/finger_gait_nc/update_obj',UpdateObj)
        obj_1(obj_pose,mesh)
        '''
        return True

# service call

# finger gait all fingers

# in-grasp manipulation 

# return generated sequence
