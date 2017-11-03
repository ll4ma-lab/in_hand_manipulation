import rospy
import tf
from subprocess import Popen
import subprocess
import numpy as np
import copy
from geometry_msgs.msg import PoseStamped,Pose

class tfAPI:
    def __init__(self):
        self.listener = tf.TransformListener()
        print "Initialized tf_api"
    def tf_pose_pub_process(self,pose,target_frame,base_frame='palm_link',rate=500):
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
