import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from in_hand_manipulation_msgs.srv import UpdateRobotConfig
from ll4ma_opt_utils.srv import AddObject
class regraspClient(object):
    def __init__(self,mesh,init_node = False):
        if(init_node):
            rospy.init_node('regrasp_client')

        self.obj_mesh = mesh
        self.j_names =  ['lbr4_j0', 'lbr4_j1', 'lbr4_j2', 'lbr4_j3', 'lbr4_j4', 'lbr4_j5',
                         'lbr4_j6',
                         'index_joint_0', 'index_joint_1','index_joint_2', 'index_joint_3',
                         'middle_joint_0', 'middle_joint_1', 'middle_joint_2', 'middle_joint_3',
                         'ring_joint_0', 'ring_joint_1', 'ring_joint_2', 'ring_joint_3',
                         'thumb_joint_0', 'thumb_joint_1','thumb_joint_2', 'thumb_joint_3']


    def update_sim_state(self,joint_config):
        rospy.wait_for_service('/in_hand_utils/update_robot_state')
        try:
            s_call = rospy.ServiceProxy('/in_hand_utils/update_robot_state', UpdateRobotConfig)
            js = JointState()
            js.name = self.j_names
            js.position = joint_config
            resp = s_call(js)
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # update object mesh, pose
    def update_obj(self,pose,pose_offset = None, mesh = None):
        # For now
        if(pose_offset is None):
            o_pose_offset = Pose()
            o_pose_offset.orientation.w = 1.0
        else:
            o_pose_offset = pose_offset
        if(mesh is not None):
            self.obj_mesh = mesh
        rospy.wait_for_service('/in_hand/finger_gait/update_obj')
        try:
            s_call = rospy.ServiceProxy('/in_hand/finger_gait/update_obj', AddObject)
            resp = s_call(self.obj_mesh,o_pose_offset,pose)
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
        
        
    # call fingertip relocation client
    

            

