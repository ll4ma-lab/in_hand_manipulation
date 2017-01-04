# Plots desired poses of all objects

import csv
import numpy as np
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
import rospy
#from geometry_msgs.msg import PoseArray
#from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from object_list import *

def file_import(f):  
    rospy.init_node('des_poses_node')
    rate=rospy.Rate(1.0)
    g_pose_pub=rospy.Publisher("/relaxed_rigidity/goal_poses",MarkerArray,queue_size=1)
    markerArray = MarkerArray()

    trial_list=[]
    empty=0
    x=[]
    y=[]
    z=[]
    xi=[]
    yi=[]
    zi=[]

    obj_list=ObjectList(f,'')
    traj_n=len(obj_list.arr)
    print traj_n
    for i in range(len(obj_list.arr)):
        obj=obj_list.arr[i]
        des_pose=obj.des
        init_pose=obj.initial_pose
        
        marker = Marker()
        marker.header.frame_id = "/palm_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.005
        marker.scale.z = 0.005
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.position.x = des_pose[0]
        marker.pose.position.y = des_pose[1]
        marker.pose.position.z = des_pose[2]
        marker.id=i
        markerArray.markers.append(marker)
        
        marker = Marker()
        marker.header.frame_id = "/palm_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.005
        marker.scale.z = 0.005
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.position.x = init_pose[0]
        marker.pose.position.y = init_pose[1]
        marker.pose.position.z = init_pose[2]
        marker.id=i+traj_n
        
        markerArray.markers.append(marker)
        

                
    while not rospy.is_shutdown():
        g_pose_pub.publish(markerArray)
        rate.sleep()
if __name__=='__main__':
    file_import('../data/goal_poses/random_goal_poses_2.csv')
