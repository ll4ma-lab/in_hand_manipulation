# Listens to allegro joint states
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
import tf

class robotStates:
    def __init__(self,node_name='allegro_node',publish_prefix='/allegro_hand_right',listen_prefix='/allegro_hand_right',rate=60.0,listen_aruco=False):
        rospy.init_node(node_name)
        self.rate=rospy.Rate(rate)
        self.switch_cmd=rospy.Publisher(publish_prefix+'/control_type',Int8,queue_size=10)

        self.robot_joint_cmd_pub=rospy.Publisher(publish_prefix+'/joint_cmd',
                                                     JointState, queue_size=100)

        rospy.Subscriber(listen_prefix+'/joint_states',JointState,self.joint_state_cb)
        if(listen_aruco):
            listener = tf.TransformListener()
            
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

        

    def joint_state_cb(self,joint_state):
        self.joint_states=joint_state
        


    def publish_grasp(self,joint_pos_arr):
        self.jc.position=joint_pos_arr
        while not rospy.is_shutdown():
            for i in range(10):
                self.robot_joint_cmd_pub.publish(self.jc)
                self.loop_rate.sleep()
            break
    def publish_joint_trajectory(self,joint_pos_traj):
        print len(joint_pos_traj)
        while not rospy.is_shutdown():
            for i in range(len(joint_pos_traj)):
                self.jc.position=joint_pos_traj[i]
                self.robot_joint_cmd_pub.publish(self.jc)
                self.loop_rate.sleep()
            break
        print 'Finished running trajectory'

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
        
