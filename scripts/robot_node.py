# Listens to allegro joint states
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
class jointListener:
    ''' This class listens to a joint state topic and gives back position/velocity/effort readings'''
    def __init__(self,node_name='allegro_joint_listener',listen_prefix='/allegro_hand_right'):
        rospy.init_node(node_name)
        rate = rospy.Rate(100) # 100hz
        print 'Initialized robot joint state listening node'
        rospy.Subscriber(listen_prefix+'/joint_states',JointState,self.joint_state_cb)
        #$rospy.spin()
    def joint_state_cb(self,joint_state):
        self.joint_states=joint_state
        

class jointPublisher:
    def __init__(self,node_name='allegro_joint_publisher',publish_prefix='/allegro_hand_right',rate=100.0):
        rospy.init_node(name=node_name)
        self.robot_joint_cmd_pub=rospy.Publisher(publish_prefix+'/joint_cmd',
                                                     JointState, queue_size=100)
        print 'Initialized robot joint state publishing node'
        self.loop_rate=rospy.Rate(rate)
        jc=JointState()
        jc.name = ['index_joint_0','index_joint_1','index_joint_2', 'index_joint_3',
                   'middle_joint_0','middle_joint_1','middle_joint_2', 'middle_joint_3',
                   'ring_joint_0','ring_joint_1','ring_joint_2', 'ring_joint_3',
                   'thumb_joint_0','thumb_joint_1','thumb_joint_2', 'thumb_joint_3']

        jc.position=[ 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0]
        self.jc=jc

    def publish_joint_positions(self,joint_pos_arr):
        self.jc.position=joint_pos_arr
        while not rospy.is_shutdown():
            for i in range(100):
                self.robot_joint_cmd_pub.publish(self.jc)
                self.loop_rate.sleep()
            break

class controlSwitch:
    def __init__(self,node_name='allegro_controller_switcher',publish_prefix='/allegro_hand_right',rate=100.0):
        rospy.init_node(node_name)
        self.rate=rospy.Rate(rate)
        self.switch_cmd=rospy.Publisher(publish_prefix+'/control_type',Int8,queue_size=10)
    def grav_comp_mode(self):
        sw_value=Int8()
        sw_value.data=2
        while not rospy.is_shutdown():
            for i in range(100):
                self.switch_cmd.publish(sw_value)
                self.rate.sleep()
            break
    def position_mode(self):
        sw_value=Int8()
        sw_value.data=0
        while not rospy.is_shutdown():
            for i in range(100):
                self.switch_cmd.publish(sw_value)
                self.rate.sleep()
            break


class robotStates:
    def __init__(self,node_name='allegro_node',publish_prefix='/allegro_hand_right',listen_prefix='/allegro_hand_right',rate=100.0):
        rospy.init_node(node_name)
        self.rate=rospy.Rate(rate)
        self.switch_cmd=rospy.Publisher(publish_prefix+'/control_type',Int8,queue_size=10)

        self.robot_joint_cmd_pub=rospy.Publisher(publish_prefix+'/joint_cmd',
                                                     JointState, queue_size=100)
        print 'Initialized robot node'
        self.loop_rate=rospy.Rate(rate)
        jc=JointState()
        jc.name = ['index_joint_0','index_joint_1','index_joint_2', 'index_joint_3',
                   'middle_joint_0','middle_joint_1','middle_joint_2', 'middle_joint_3',
                   'ring_joint_0','ring_joint_1','ring_joint_2', 'ring_joint_3',
                   'thumb_joint_0','thumb_joint_1','thumb_joint_2', 'thumb_joint_3']

        jc.position=[ 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0]
        self.jc=jc

        rospy.Subscriber(listen_prefix+'/joint_states',JointState,self.joint_state_cb)

    def joint_state_cb(self,joint_state):
        self.joint_states=joint_state
        

    def publish_joint_positions(self,joint_pos_arr):
        self.jc.position=joint_pos_arr
        while not rospy.is_shutdown():
            for i in range(100):
                self.robot_joint_cmd_pub.publish(self.jc)
                self.loop_rate.sleep()
            break        
    def grav_comp_mode(self):
        sw_value=Int8()
        sw_value.data=2
        while not rospy.is_shutdown():
            for i in range(100):
                self.switch_cmd.publish(sw_value)
                self.rate.sleep()
            break
    def position_mode(self):
        sw_value=Int8()
        sw_value.data=0
        while not rospy.is_shutdown():
            for i in range(100):
                self.switch_cmd.publish(sw_value)
                self.rate.sleep()
            break
        
