import rospy
import tf
from subprocess import Popen
import subprocess
import numpy as np
import copy
from geometry_msgs.msg import PoseStamped,Pose
import matplotlib.pyplot as plt
twoui=['#fc8d59','#d7301f']
params = {'legend.fontsize': 'x-large',
          'figure.figsize': (15, 5),
         'axes.labelsize': 'x-large',
         'axes.titlesize':'x-large',
         'xtick.labelsize':'x-large',
         'ytick.labelsize':'x-large'}
plt.rcParams.update(params)
class tfAPI:
    def __init__(self):
        self.listener = tf.TransformListener()
        print "Initialized tf_api"
    def tf_pose_pub_process(self,pose,target_frame,base_frame='palm_link',rate=50):
        tf_pose=tf.TransformBroadcaster()
        pose=pose.tolist()
        if(len(pose)==3):
            pose.extend([0.0,0.0,0.0])
        quat=tf.transformations.quaternion_from_euler(pose[3],pose[4],pose[5])
        #x y z qx qy qz qw frame_id child_frame_id  period_in_ms
        args=[str(pose[0]), str(pose[1]), str(pose[2]), str(quat[0]), str(quat[1]), str(quat[2]), str(quat[3]), base_frame, target_frame,str(rate)]
        cmd=['/bin/bash','rosrun','tf','static_transform_publisher']+args
        proc=Popen(cmd,stdout=subprocess.PIPE)
        return proc
    
    def tf_find_transform(self,target_frame,base_frame='palm_link'):
        got_tf=False
        time=copy.deepcopy(rospy.Time(0))
        self.listener.waitForTransform(base_frame,target_frame, rospy.Time(), rospy.Duration(4.0))

        (trans_2,rot_2)= self.listener.lookupTransform(base_frame, target_frame, rospy.Time(0))
        final_pose=np.zeros(6)
        final_pose[0:3]=trans_2
        final_pose[3:6]=tf.transformations.euler_from_quaternion(rot_2)
        got_tf=True    
        return final_pose

class poseAPI:
    def __init__(self):
        print "poseAPI initialized"
    def pose_from_arr(self,u):
        pose=Pose()
        pose.position.x=u[0]
        pose.position.y=u[1]
        pose.position.z=u[2]
        
        quaternion = tf.transformations.quaternion_from_euler(u[3],u[4],u[5])
        pose.orientation.x=quaternion[0]
        pose.orientation.y=quaternion[1]
        pose.orientation.z=quaternion[2]
        pose.orientation.w=quaternion[3]
        pose_st=PoseStamped()
        pose_st.pose=pose
        return pose_st
    def pose_to_arr(self,u):
        pose_arr=np.zeros(6)
        pose_arr[0]=u.position.x
        pose_arr[1]=u.position.y
        pose_arr[2]=u.position.z
        
        pose_arr[3:6]=tf.transformations.euler_from_quaternion([u.orientation.x,u.orientation.y,u.orientation.z,u.orientation.w])
        return pose_arr

    def quat_from_rpy(self,u):
        quaternion = tf.transformations.quaternion_from_euler(u[0],u[1],u[2])
        return [quaternion[0],quaternion[1],quaternion[2],quaternion[3]]
    def plot_acc_profiles(self,pose_arr_list,labels):
        style=['-','--']
        for i in range(len(pose_arr_list)):
            acc_,vel_=self.compute_acceleration(pose_arr_list[i])
            #plt.subplot(2, 1, 1)
            plt.plot(vel_,style[i],label=labels[i],linewidth=4,color=twoui[i])
            plt.xlabel('Timestep')
            plt.ylabel('l-2 norm velocity ($cm s^{-1}$)')
          
            '''
            plt.subplot(2, 1, 2)
            plt.plot(acc_,style[i],label=labels[i],linewidth=4,color=twoui[i])
            plt.ylim(ymin=0.0)
            plt.xlabel('Timesteps(s)')
            plt.ylabel('l-2 norm acceleration ($cm s^{-2}$)')
            '''
        plt.ylim(ymin=0.0)
        #plt.subplot(2, 1, 1)
        plt.grid()
        plt.legend(loc='upper center',bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
        #plt.subplot(2, 1, 2)

        plt.show()
    def compute_acceleration(self,pose_arr):
        pos_arr=[]
        for j in range(len(pose_arr.poses)):
            pos=np.array([pose_arr.poses[j].position.x,pose_arr.poses[j].position.y, pose_arr.poses[j].position.z])
            pos_arr.append(pos)

        acc_prof=[]
        vel_prof=[]
        for t in range(1,len(pos_arr)-1):
            acc=pos_arr[t-1]-2.0*pos_arr[t]+pos_arr[t+1]
            acc_prof.append(np.linalg.norm(acc,2)*100.0)
            
        for t in range(1,len(pos_arr)):
            vel=-pos_arr[t-1]+pos_arr[t]
            vel_prof.append(np.linalg.norm(vel,2)*100.0)

        return acc_prof,vel_prof

