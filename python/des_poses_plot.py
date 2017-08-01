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


'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
'''
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

    with open(f,'r') as csvfile:
        csvreader=csv.reader(csvfile,delimiter=',',quotechar='"')
        line=1
        init_flag=False
        # Create object class
        for row in csvreader:
            if(csvreader.line_num>0):
                des_pose=np.array(map(float,row[4:10]))
                init_pose=np.array(map(float,row[10:16]))

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
                if(csvreader.line_num<6):
                    marker.color.g = 0.0
                marker.id=csvreader.line_num
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
                marker.id=csvreader.line_num+100
                
                markerArray.markers.append(marker)


                '''
                x.append(des_pose[0])
                y.append(des_pose[1])
                z.append(-des_pose[2])
                xi.append(init_pose[0])
                yi.append(init_pose[1])
                zi.append(-init_pose[2])
                '''

                
        while not rospy.is_shutdown():
            g_pose_pub.publish(markerArray)
            rate.sleep()
        
        '''
        ax.scatter(x, z, y, zdir='z', s=100, c='red', depthshade=False)
        ax.scatter(xi, zi, yi, zdir='z', s=100, c='blue', depthshade=False)
        ax.scatter(0.0,0.0, 0.0, zdir='z', s=100, c='green', depthshade=False)
        ax.set_xlabel('x')
        ax.set_ylabel('z')
        ax.set_zlabel('y')
        
        print len(xi)
        plt.show()
        print des_pose
        print init_pose
        '''
if __name__=='__main__':
    file_import('ral_poses_centroid_new_exp.csv')
