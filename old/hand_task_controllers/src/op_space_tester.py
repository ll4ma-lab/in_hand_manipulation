#!/usr/bin/env python
import rospy
from std_msgs.msg import Char
#from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import numpy as np

class JointTester:

    def __init__(self, node_name='joint_tester', listen_prefix='', listen=False, publish_prefix=''):
        rospy.init_node(node_name)

        self.allegro_point_cmd_pub=rospy.Publisher(publish_prefix+'/point_cmd',PointStamped,queue_size=10)

        run_rate = rospy.Rate(100)
        self.init_time=rospy.get_time()


    def run_allegro_op_position_test(self):
        run_rate = rospy.Rate(100)

        pt=PointStamped()
        pt.point.x=1.0
        pt.point.y=0.0
        pt.point.z=-1.0

        while not rospy.is_shutdown():
            pt.header.stamp=rospy.Time.now();
            self.allegro_point_cmd_pub.publish(pt)
            run_rate.sleep()

if __name__=='__main__':
    listen=False
    listen_prefix='allegro_hand_right'
    publish_prefix='allegro_hand_right'
    jt = JointTester(listen=listen, listen_prefix=listen_prefix, publish_prefix=publish_prefix)
    rospy.loginfo('Running allegro control test')

    jt.run_allegro_op_position_test()
