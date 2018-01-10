#!/usr/bin/env python
import rospy

from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped,PointStamped
import numpy as np
from viz_markers import VizInHand
import sys
import time
import pickle
from numpy.linalg import inv
import PyKDL
import copy
def get_T_mat(pose):
    T=np.eye(4)
    T[0:3,3]=np.ravel([pose.position.x,pose.position.y,pose.position.z])
    #
    R=PyKDL.Rotation()
    R=R.Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
    for k in range(3):
        for l in range(3):
            T[k,l]=R[k,l]

    return T
def get_T_mat_pt(pt):
    T=np.eye(4)
    T[0:3,3]=np.ravel([pt.x,pt.y,pt.z])
    return T


if __name__=='__main__':
    grasp_loc='/home/bala/catkin_ws/src/grasp_gaits'
    #o_mesh='file://'+grasp_loc+'/data/object_models/banana_bin.stl'
    o_mesh='file::///home/bala/object_models/banana/google_16k/nontextured.stl'
    cs_meshes=['file://'+grasp_loc+'/scripts/f0.stl',
              'file://'+grasp_loc+'/scripts/f1.stl',
              'file://'+grasp_loc+'/scripts/f2.stl',
              'file://'+grasp_loc+'/scripts/f3.stl']
    
    f_meshes=['file://'+grasp_loc+'/data/robot_models/voxel_index.ply',
              'file://'+grasp_loc+'/data/robot_models/voxel_middle.ply',
              'file://'+grasp_loc+'/data/robot_models/voxel_ring.ply',
              'file://'+grasp_loc+'/data/robot_models/voxel_thumb.ply']


    viz=VizInHand(o_mesh,f_meshes,t_name="finger_gait")    
    

    ## Initial grasp:
    
    
    # load grasp data:

    file_ = open(sys.argv[1],'rb')
    object_file = pickle.load(file_)
    file_.close()
    G_arr=object_file
    time.sleep(1.0)
    G=G_arr[0]
    # Visualize hand joint state
    viz.viz_js(t_name="finger_gait",js=G[1])
    obj_col=[1.0,1.0,0.0]
    # Vizualize object pose:
    viz.viz_obj_mesh(G[0],a=1.0,rgb=obj_col)
    obj_pose=object_file[0][0]
        
    d_pts=object_file[-2][2]
    d_obj_pose=object_file[-2][0]
    viz.viz_obj_mesh(d_obj_pose,a=1.0,rgb=obj_col)
    viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])
    o_T_c=[]
    # build obj_pose mat
    obj_mat=np.matrix(inv(get_T_mat(d_obj_pose.pose)))
    z_obj_mat=np.matrix(inv(get_T_mat(obj_pose.pose)))
    n_d_pts=copy.deepcopy(d_pts)
    for i in range(len(d_pts)):
        pt=d_pts[i]
        T_c=get_T_mat_pt(pt.point)
        o_T_c.append(obj_mat*T_c)
        new_pt=np.ravel((inv(z_obj_mat)*np.matrix(o_T_c[i]))[0:3,3])
        pt.point.x=new_pt[0]
        pt.point.y=new_pt[1]
        pt.point.z=new_pt[2]
        n_d_pts[i]=pt
    d_pts=n_d_pts
    viz.viz_obj_mesh(obj_pose,a=1.0,rgb=obj_col)
    viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])

    
    # Visualize contact points 
    #viz.viz_cpts(G[2],t='d_cpt',rgb=[0.0,1.0,0.0])    
    time.sleep(0.1)
    raw_input("Run plan?")
    
    for i in range(1,len(G_arr)-1):
        G=G_arr[i]
        # Visualize hand joint state
        viz.viz_js(t_name="finger_gait",js=G[1])

        # Vizualize object pose:
        viz.viz_obj_mesh(G[0],a=1.0,rgb=obj_col)
        
        
               
        obj_mat=get_T_mat(G[0].pose)
        for i in range(len(d_pts)):
            new_pt=obj_mat*np.matrix(o_T_c[i])
            d_pts[i].point.x=new_pt[0,3]
            d_pts[i].point.y=new_pt[1,3]
            d_pts[i].point.z=new_pt[2,3]


        # Visualize contact points 
        viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])    
        #time.sleep(0.1)
        raw_input("Next step?")

        
