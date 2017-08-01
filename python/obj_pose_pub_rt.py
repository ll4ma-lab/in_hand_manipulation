#!/usr/bin/env python
# This script publishes tf from kdl frames, publishing the object pose for the current finger tip states and also an additional object pose for the desired object pose.

# Usage: python obj_pose_pub_rt.py trajectory_index write_reached_pose(0/1)
from kdl_allegro_model import *
import tf
import rospy
from sensor_msgs.msg import JointState
_DISPLAY_RATE = 50
from object_list import *
import sys

def stCallback(data):
    global curr_joints
    curr_joints=np.array(data.position)
    #joint_sub=True

def main():
    tj=int(sys.argv[1])
    write_to_file=int(sys.argv[2])
    file_name="ral_rerun_poses.csv"
    folder_to_write="junk/"
    obj_list=ObjectList(file_name,folder_to_write)
        


    D=np.array(obj_list.arr[tj].offset) # Distance of the object from finger   
    
    joints_init=obj_list.arr[tj].grasp
        
    delta_t=1.0/60
    T=100
    pose_d=[0,1,2,3,4,5]
    robot_model=allegroKinematics(delta_t,T,fingers=obj_list.arr[tj].fing)
    robot_model.pose_d=pose_d
    
    robot_model.object_frames=robot_model.add_object_frame(D,joints_init)    
    start_obj=robot_model.object_pose(joints_init,finger=3)
    
    #x_des=start_obj.copy()
    global curr_joints
    curr_joints=np.zeros(16)
    x_des=np.array(obj_list.arr[tj].des)


    des_obj=x_des
    init_pose=obj_list.arr[tj].initial_pose
    print init_pose
    # Convert these object frames to tf and publish as tf msg
    rospy.init_node('object_pose_node')
    print 'created node'
    rate = rospy.Rate(60.0)
    listener = tf.TransformListener()
    rospy.Subscriber("allegro_hand_right/joint_states",JointState , stCallback)
    print_rate=1
    final_pose=[]
    obj_initial_flag=False
    trans1=[]
    rot1=[]
    while not rospy.is_shutdown():
        print_rate+=1
        initial_pose = tf.TransformBroadcaster()
        initial_pose.sendTransform((init_pose[0],init_pose[1], init_pose[2]),
                                   tf.transformations.quaternion_from_euler(init_pose[3], init_pose[4], init_pose[5]),
                                   rospy.Time.now(),
                                   "i",
                                   "palm_link")
        try:
            if(obj_initial_flag==False):
                (trans1,rot1)= listener.lookupTransform('/ar', '/i', rospy.Time(0))            
            obj_initial_flag=True
        except: 
            obj_initial_flag=False
            continue
        
        if(obj_initial_flag==True):
            obj_pose = tf.TransformBroadcaster()
            obj_pose.sendTransform(trans1,rot1, rospy.Time.now(),"/obj","/ar")

        goal_pose = tf.TransformBroadcaster()
        goal_obj=robot_model.object_pose(curr_joints,finger=3)
        goal_pose.sendTransform((goal_obj[0],goal_obj[1], goal_obj[2]),
                                tf.transformations.quaternion_from_euler(goal_obj[3], goal_obj[4], goal_obj[5]),                         rospy.Time.now(),
                                "R",
                                "palm_link")            
        des_pose = tf.TransformBroadcaster()

        des_pose.sendTransform((des_obj[0],des_obj[1], des_obj[2]),
                                tf.transformations.quaternion_from_euler(des_obj[3], des_obj[4], des_obj[5]),                         rospy.Time.now(),
                                "G",
                                "palm_link")            

        try:
            (trans_2,rot_2)= listener.lookupTransform('/palm_link', '/obj', rospy.Time(0))            
            final_pose=np.zeros(6)
            final_pose[0:3]=trans_2
            
            final_pose[3:6]=tf.transformations.euler_from_quaternion(rot_2)
        except:
            continue
        rate.sleep()

    obj_list.arr[tj].sol=final_pose
    print final_pose

    if(write_to_file==1):
        obj_list.write_sol(file_name)

if __name__=='__main__':
    main()
