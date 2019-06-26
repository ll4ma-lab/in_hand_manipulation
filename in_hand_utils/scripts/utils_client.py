import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray

class inHandUtils(object):
    def __init__(self,init_node=False):
        if(init_node):
            rospy.init_node('in_hand_utils')

        self.got_sn=False
        self.sn=[]
        # subscribers:
        rospy.Subscriber('/in_hand_utils/surface_normal',MarkerArray,self.sn_cb)
        self.loop_rate=rospy.Rate(100.0)
    def sn_cb(self,msg):
        self.sn=[]
        for i in range(len(msg.markers)):
            pt1=np.array([msg.markers[i].points[0].x,msg.markers[i].points[0].y,msg.markers[i].points[0].z])
            pt2=np.array([msg.markers[i].points[1].x,msg.markers[i].points[1].y,msg.markers[i].points[1].z])
            
            sn=(pt2-pt1)/np.linalg.norm(pt2-pt1)
            self.sn.append(sn)
        self.got_sn=True
    
    def get_sn(self):
        while(not self.got_sn):
            self.loop_rate.sleep()
        self.got_sn=False
        return self.sn
