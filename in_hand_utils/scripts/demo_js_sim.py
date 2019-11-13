import rospy

from sensor_msgs.msg import JointState
from in_hand_manipulation_msgs.srv import UpdateRobotConfig

def update_sim_state(joint_config,j_names):
    rospy.wait_for_service('/in_hand_utils/update_robot_state')
    try:
        s_call = rospy.ServiceProxy('/in_hand_utils/update_robot_state', UpdateRobotConfig)
        js = JointState()
        js.name = j_names
        js.position = joint_config
        resp = s_call(js)
        return resp.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
 

if __name__=='__main__':
    rospy.init_node('test_js')
    j_names = ['lbr4_j0', 'lbr4_j1', 'lbr4_j2', 'lbr4_j3', 'lbr4_j4', 'lbr4_j5', 'lbr4_j6',
               'index_joint_0', 'index_joint_1','index_joint_2', 'index_joint_3',
               'middle_joint_0', 'middle_joint_1', 'middle_joint_2', 'middle_joint_3',
               'ring_joint_0', 'ring_joint_1', 'ring_joint_2', 'ring_joint_3',
               'thumb_joint_0', 'thumb_joint_1','thumb_joint_2', 'thumb_joint_3']

    j_config = [0.65 for i in range(len(j_names))]

    j_config[0]=0.0
    j_config[1]=1.4
    j_config[2]=0.0
    j_config[3]=0.0
    j_config[4]=0.0
    j_config[5]=0.0


    print update_sim_state(j_config,j_names)
   
