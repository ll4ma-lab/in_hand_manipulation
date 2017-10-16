# Publishes trajectory of an object in rviz as a marker.
import rospy
from std_msgs.msg import Char
from sensor_msgs.msg import JointState
import numpy as np
from object_list import *
import sys
from kdl_allegro_model import *

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

delta_t=1.0/60
T=10

def pub_traj(obj1,obj2):
    rospy.init_node('obj_traj_pub')
    rate=rospy.Rate(1.0)
    linear_pub=rospy.Publisher("/object_linear/traj",MarkerArray,queue_size=1)
    linear_arr=MarkerArray()
    no_linear_arr=MarkerArray()
    no_linear_pub=rospy.Publisher("/object/traj",MarkerArray,queue_size=1)
    sample_arr=MarkerArray()
    sample_pub=rospy.Publisher("/object/sample",MarkerArray,queue_size=1)
    f1=open(obj1.folder+obj1.f_name,'r')
    f2=open(obj1.folder+obj2.f_name,'r')
    joint_arr1=np.loadtxt(f1)
    joint_arr2=np.loadtxt(f2)
    # loading object model:
    D=obj1.offset # Distance of the object from finger  

    # initial joint angles    
    joints=obj1.grasp

    robot_model=allegroKinematics(delta_t,T,fingers=obj1.fing)
    robot_model.joints=joints
    robot_model.object_frames=robot_model.add_object_frame(D,joints,finger=3)              
    
    x0=robot_model.object_pose(joints,3)
    x_des=obj1.des
    
    linear_poses=[]
    for i in range(T):
        sample_pose=x0+(x_des-x0)*float(i)/T
        # Building transformation matrix:
        x_des_mat=np.eye(4)
        x_des_mat[0:3,3]=sample_pose[0:3]
        R=PyKDL.Rotation.RPY(sample_pose[3],sample_pose[4],sample_pose[5])
        for i in range(3):
            for j in range(3):
                x_des_mat[i,j]=R[i,j]
        obj_pose=np.zeros(7)
        obj_pose[0:3]=sample_pose[0:3]
        obj_pose[3:7]=R.GetQuaternion()
        linear_poses.append(obj_pose)



    for i in range(0,len(joint_arr1),10):
        obj_pose1=robot_model.object_pose(joint_arr1[i],3,get_quat=True)
        obj_pose2=robot_model.object_pose(joint_arr2[i],3,get_quat=True)
        lin_pose=linear_poses[i/10]
        marker=Marker()

        marker.header.frame_id = "/palm_link"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.003
        marker.scale.y = 0.010
        marker.scale.z = 0.003
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.position.x = lin_pose[0]
        marker.pose.position.y = lin_pose[1]
        marker.pose.position.z = lin_pose[2]
        marker.pose.orientation.x = lin_pose[3]
        marker.pose.orientation.y = lin_pose[4]
        marker.pose.orientation.z = lin_pose[5]
        marker.pose.orientation.w = lin_pose[6]
        marker.id=i
        sample_arr.markers.append(marker)


        marker=Marker()

        marker.header.frame_id = "/palm_link"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.003
        marker.scale.y = 0.010
        marker.scale.z = 0.003
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = obj_pose1[0]
        marker.pose.position.y = obj_pose1[1]
        marker.pose.position.z = obj_pose1[2]
        marker.pose.orientation.x = obj_pose1[3]
        marker.pose.orientation.y = obj_pose1[4]
        marker.pose.orientation.z = obj_pose1[5]
        marker.pose.orientation.w = obj_pose1[6]
        marker.id=i
        linear_arr.markers.append(marker)

        marker=Marker()
        marker.header.frame_id = "/palm_link"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.003
        marker.scale.y = 0.010
        marker.scale.z = 0.003
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.position.x = obj_pose2[0]
        marker.pose.position.y = obj_pose2[1]
        marker.pose.position.z = obj_pose2[2]
        marker.pose.orientation.x = obj_pose2[3]
        marker.pose.orientation.y = obj_pose2[4]
        marker.pose.orientation.z = obj_pose2[5]
        marker.pose.orientation.w = obj_pose2[6]
        marker.id=i
        no_linear_arr.markers.append(marker)
    while not rospy.is_shutdown():
        linear_pub.publish(linear_arr)
        no_linear_pub.publish(no_linear_arr)
        sample_pub.publish(sample_arr)
        rate.sleep()
    
if __name__=='__main__':
    traj_no=int(sys.argv[1])
    file_name="../data/trajectories/waypoint_solution.csv"
    folder_to_write="../data/trajectories/waypoint/"
    obj=TrajList(file_name,folder_to_write)
    pub_traj(obj.arr[traj_no],obj.arr[traj_no+50])
    
