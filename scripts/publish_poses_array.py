# Plots desired poses of all objects

import csv
import numpy as np
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
import rospy
#from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray
from object_list import *
import tf
def file_import(f):  
    rospy.init_node('des_poses_node')
    rate=rospy.Rate(1.0)
    base_frame= "/palm_link"
    tf_pose=tf.TransformBroadcaster()

    
    obj_list=ObjectList(f,'')
    traj_n=len(obj_list.arr)
    print traj_n
    while not rospy.is_shutdown():
    
        for i in range(len(obj_list.arr)):
            obj=obj_list.arr[i]
            des_pose=obj.des
            init_pose=obj.initial_pose
            target_frame="des_"+str(i)
            tf_pose.sendTransform((des_pose[0],des_pose[1],des_pose[2]),tf.transformations.quaternion_from_euler(des_pose[3],des_pose[4],des_pose[5]),rospy.Time.now(),target_frame,base_frame)
        
                
        
        rate.sleep()
if __name__=='__main__':
    file_import('../data/goal_poses/feas_goal_poses.csv')
