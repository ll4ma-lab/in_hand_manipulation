#!/usr/bin/env python
import rospy
from std_msgs.msg import Char
#from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import numpy as np

class JointTester:

    def __init__(self, node_name='joint_tester', listen_prefix='', listen=False, publish_prefix=''):
        rospy.init_node(node_name)

        self.allegro_point_cmd_pub=rospy.Publisher(publish_prefix+'/force_cmd',PoseArray,queue_size=10)

        run_rate = rospy.Rate(100)
        self.init_time=rospy.get_time()


    def run_allegro_op_force_test(self):
        run_rate = rospy.Rate(100)
        
        pose_arr=PoseArray()


        for i in range(3):
            p_=Pose()
            p_.position.x=0.0
            p_.position.y=0.0
            p_.position.z=-2.0
            pose_arr.poses.append(p_)

        p_=Pose()
        p_.position.x=0.0
        p_.position.y=0.0
        p_.position.z=2.0
        pose_arr.poses.append(p_)
        

        while not rospy.is_shutdown():
            
            self.allegro_point_cmd_pub.publish(pose_arr)
            run_rate.sleep()

if __name__=='__main__':
    listen=False
    listen_prefix='allegro_hand_right'
    publish_prefix='allegro_hand_right'
    jt = JointTester(listen=listen, listen_prefix=listen_prefix, publish_prefix=publish_prefix)
    rospy.loginfo('Running allegro control test')

    jt.run_allegro_op_force_test()
