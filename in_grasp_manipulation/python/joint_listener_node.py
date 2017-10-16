# Listens to allegro joint states
import rospy
from sensor_msgs.msg import JointState

class jointListener:
    ''' This class listens to a joint state topic and gives back position/velocity/effort readings'''
    def __init__(self,node_name='allegro_joint_listener',listen_prefix='/allegro'):
        rospy.init_node(node_name)
        print 'Initialized robot joint state listening node'
        rospy.Subscriber(listen_prefix+'/joint_states',JointState,self.joint_state_cb)
        rospy.spin()
    def joint_state_cb(self,joint_state):
        self.joint_states=joint_state
        
