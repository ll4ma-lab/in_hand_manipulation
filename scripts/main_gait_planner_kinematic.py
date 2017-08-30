#!/usr/bin/env python
import rospy
from in_hand_manipulation.srv import GetHandIK,GetHandContact
from grasp_gaits.srv import GetEllipse,RelocateFingerKin,RelocateObjSd
from in_grasp_manipulation.srv import InGraspManipulationSd
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped,PointStamped
import numpy as np
from viz_markers import VizInHand
import copy
import PyKDL
from numpy.linalg import inv
import sys
from rospkg import RosPack
rp=RosPack()
rp.list()
path_g=rp.get_path('in_grasp_manipulation')
sys.path.insert(0,path_g+'/scripts')

import in_grasp_client

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

class handIKClient:
    def __init__(self):
        print "Yay! initialized hand ik client"
        
    def call(self,f_index,des_pt,js):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)

        d_poses=[]
        for i in range(len(des_pt)):
            p=Pose()
            p.position.x=des_pt[i].point.x
            p.position.y=des_pt[i].point.y
            p.position.z=des_pt[i].point.z
            p.orientation.x=0.0
            p.orientation.y=0.0
            p.orientation.z=0.0
            p.orientation.w=1.0
            d_poses.append(p)

        rospy.wait_for_service('/in_hand/ik')
        get_ik=rospy.ServiceProxy('/in_hand/ik',GetHandIK)
        j_=get_ik(f_,js,d_poses,False)
        
        return j_.joint_state

    def tester(self,f_index=[2,1]):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)
        js=JointState()
        js.position=np.zeros(16)

        d_poses=[]
        for i in range(4):
            p=Pose()
            p.position.x=0.03
            p.position.y=0.0
            p.position.z=0.03
            p.orientation.x=0.0
            p.orientation.y=0.0
            p.orientation.z=0.0
            p.orientation.w=1.0
            d_poses.append(p)

        rospy.wait_for_service('/in_hand/ik')
        get_ik=rospy.ServiceProxy('/in_hand/ik',GetHandIK)
        j_=get_ik(f_,js,d_poses,False)
        
        return j_.joint_state.position

class handContactClient:
    def __init__(self):
        print "hand contact client: Yay!"

    def call(self,obj_pose,js,f_index,transform_points,initial_obj_pose):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)
        rospy.wait_for_service('/in_hand_manipulation/get_hand_contacts')
        srv_fn=rospy.ServiceProxy('/in_hand_manipulation/get_hand_contacts',GetHandContact)
        res=srv_fn(obj_pose,js,f_,transform_points,initial_obj_pose)
        #print res.vertex_idx_arr
        #print res.contact_pts
        return res.vertex_idx_arr,res.contact_pts

    def tester(self,f_index=[1,2]):
        p=PoseStamped()
        p.pose.position.x=0.08
        p.pose.position.y=-0.08
        p.pose.position.z=0.015
        p.pose.orientation.x=-0.706825
        p.pose.orientation.y=0.0
        p.pose.orientation.z=0.0
        p.pose.orientation.w=0.707

        js=JointState()
        js.position=np.zeros(16)

        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)
        rospy.wait_for_service('/in_hand_manipulation/get_hand_contacts')
        srv_fn=rospy.ServiceProxy('/in_hand_manipulation/get_hand_contacts',GetHandContact)
        res=srv_fn(p,js,f_)
        #print res.vertex_idx_arr
        #print res.contact_pts
        return res.vertex_idx_arr,res.contact_pts

class relocateClient:
    def __init__(self):
        print 'Relocate Client: Yay! I am ready'
    def call(self,obj_pose,c_pts,f_,des_cpt,js):
        rospy.wait_for_service('/regrasp_planner/relocate_finger')
        srv_fn=rospy.ServiceProxy('/regrasp_planner/relocate_finger',RelocateFingerKin)
        res=srv_fn(obj_pose,c_pts,f_,des_cpt,js)
        return res.next_contact_pt,res.n_js,res.stability_score

class relocateObjClient:
    def __init__(self):
        print "Relocate Object client: Ready!!"

    def call(self,f_index,des_cpts,obj_pose,c_pts):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)
        rospy.wait_for_service('/regrasp_planner/relocate_object')
        srv_fn=rospy.ServiceProxy('/regrasp_planner/relocate_object',RelocateObjSd)
        res=srv_fn(f_,obj_pose,des_cpts,c_pts)
        return res.n_obj_pose
class inGraspSdClient:
    def __init__(self):
        print 'inGraspSdClient: Ready!'
    def call(self,f_index,des_pts,obj_pose,js,T):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)
        rospy.wait_for_service('in_hand/primitive/in_grasp_manipulation')
        srv_fn=rospy.ServiceProxy('in_hand/primitive/in_grasp_manipulation',InGraspManipulationSd)
        res=srv_fn(js,obj_pose,des_pts,f_,T)
        return res.n_js, res.n_obj_pose


if __name__=='__main__':
    grasp_loc='/home/bala/catkin_ws/src/grasp_gaits'
    o_mesh='file://'+grasp_loc+'/data/object_models/banana_bin.stl'
    cs_meshes=['file://'+grasp_loc+'/scripts/f0.stl',
              'file://'+grasp_loc+'/scripts/f1.stl',
              'file://'+grasp_loc+'/scripts/f2.stl',
              'file://'+grasp_loc+'/scripts/f3.stl']
    
    f_meshes=['file://'+grasp_loc+'/data/robot_models/voxel_index.ply',
              'file://'+grasp_loc+'/data/robot_models/voxel_middle.ply',
              'file://'+grasp_loc+'/data/robot_models/voxel_ring.ply',
              'file://'+grasp_loc+'/data/robot_models/voxel_thumb.ply']


    viz=VizInHand(o_mesh,f_meshes)    
    

    ## Initial grasp:
    # Get object pose
    obj_pose=PoseStamped()
    obj_pose.pose.position.x=0.06
    obj_pose.pose.position.y=0.06
    obj_pose.pose.position.z=0.00
    obj_pose.pose.orientation.x=0.0
    obj_pose.pose.orientation.y=0.1
    obj_pose.pose.orientation.z=0.707
    obj_pose.pose.orientation.w=0.707

    f_index=[0,1,2,3]
    # Get allegro hand joint angles
    js=JointState()
    js.position=np.ones(16)*0.3
    #js.position[1]=0.5
    js.position[12]=1.3
    js.position[14]=0.3
    js.position[15]=0.8
    # Get initial contact points:
    hand_contact=handContactClient()
    _,c_pts=hand_contact.call(obj_pose,js,f_index,transform_points=False,initial_obj_pose=obj_pose)
    initial_obj_pose=copy.deepcopy(obj_pose)

    # Get initial hand joint configuration:
    hand_ik=handIKClient()
    #for i in range(4):
    n_js=hand_ik.call(f_index,c_pts,js)
    #print n_js
    
    viz.viz_js(t_name="finger_gait",js=n_js)
    # Desired contact points:
    obj_pose.pose.position.y-=0.10
    #obj_pose.pose.position.x-=0.02
    js.position[12]=1.5
    #js.position[15]=0.8
    js.position[14]=0.1
    _,d_pts=hand_contact.call(obj_pose, js, f_index=[0,1,2,3], transform_points=True, initial_obj_pose=initial_obj_pose)
    
    # Build obj_contact points:
    obj_pose.pose.position.y+=0.10
    #obj_pose.pose.position.x+=0.02

    # Build transform matrices for contact points:
    ## desired contact points:
    o_T_c=[]
    # build obj_pose mat
    obj_mat=np.matrix(inv(get_T_mat(obj_pose.pose)))
    for pt in d_pts:
        T_c=get_T_mat_pt(pt.point)
        o_T_c.append(obj_mat*T_c)
        
    # Visualize initial grasp:
    viz.viz_obj_mesh(obj_pose,a=1.0,rgb=[1.0,1.0,0.0])
    viz.viz_cpts(c_pts,t='cpt',rgb=[1.0,0.0,0.0])
    viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])    

    k=0
    thresh=0.003
    relocate_client=relocateClient()
    in_grasp_sd_client=inGraspSdClient()
    #relocate_obj_client=relocateObjClient()
    #in_grasp_obj_client=in_grasp_client.inGraspClient()
    js=n_js
    #n_obj_pose=relocate_obj_client.call(f_index,d_pts,obj_pose,c_pts)
    #viz.viz_obj_mesh(n_obj_pose,a=1.0,rgb=[1.0,0.0,0.0])
    #exit()

    while(k<10):

        # Compute finger gait contact location
        for i in f_index:
            f=i
            _,js,_=relocate_client.call(obj_pose,c_pts,f,d_pts[f],js)

        viz.viz_js(t_name="finger_gait",js=js)
        _,c_pts=hand_contact.call(obj_pose,js,f_index,transform_points=False,initial_obj_pose=obj_pose)
        diff=np.zeros(4)
        for i in range(len(d_pts)):
            diff[i]=(np.abs(d_pts[i].point.x-c_pts[i].point.x)+
                     np.abs(d_pts[i].point.y-c_pts[i].point.y)+np.abs(d_pts[i].point.z-c_pts[i].point.z))

        print diff
        if((diff < thresh).all()==True):
            print diff
            break


        viz.viz_cpts(c_pts,t='cpt',rgb=[1.0,0.0,0.0])



        #raw_input("Relocated!")

        # Relocate object to move workspace:
        # inputs: f_index,contact points, ellipses, object pose
        # outputs: object_pose
        js,n_obj_pose=in_grasp_sd_client.call(f_index,d_pts,obj_pose,js,T=1)

        #n_obj_pose=relocate_obj_client.call(f_index,d_pts,obj_pose,c_pts)
        #viz.viz_obj_mesh(n_obj_pose,t='svd',a=0.7,rgb=[1.0,0.0,0.0])
        #raw_input("found new object pose")
        #js,n_obj_pose=in_grasp_obj_client.call_server(js,obj_pose.pose,n_obj_pose.pose,f_index)

        obj_mat=get_T_mat(n_obj_pose.pose)
        for i in range(len(d_pts)):
            new_pt=obj_mat*o_T_c[i]
            d_pts[i].point.x=new_pt[0,3]
            d_pts[i].point.y=new_pt[1,3]
            d_pts[i].point.z=new_pt[2,3]
        
        obj_pose=n_obj_pose


        viz.viz_obj_mesh(obj_pose,a=1.0,rgb=[1.0,1.0,0.0])
        viz.viz_js(t_name="finger_gait",js=js)
        viz.viz_cpts(c_pts,t='cpt',rgb=[1.0,0.0,0.0])
        viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])

        #raw_input("next opt?")
        k+=1
    
