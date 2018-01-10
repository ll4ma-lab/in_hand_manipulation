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
import time
rp=RosPack()
rp.list()
path_g=rp.get_path('in_grasp_manipulation')
sys.path.insert(0,path_g+'/scripts')
path_=rp.get_path('optimization_pkg')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *

import in_grasp_client
import frames
import pickle

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
        res=srv_fn(obj_pose,c_pts,f_,des_cpt,js,10)
        return res.next_contact_pt,res.n_js,res.stability_score,res.joint_traj

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
    '''
    def call(self,f_index,des_pts,obj_pose,js,T):
        f_=f_index
        rospy.wait_for_service('in_hand/primitive/in_grasp_manipulation')
        srv_fn=rospy.ServiceProxy('in_hand/primitive/in_grasp_manipulation',InGraspManipulationSd)
        des_obj_pose=copy.deepcopy(obj_pose)
        #des_obj_pose.pose.position.y-=0.01
        #des_obj_pose.pose.position.z-=0.02
        #des_obj_pose.pose.orientation.x+=0.1
        res=srv_fn(js,obj_pose,des_pts,f_,T,[1000.0,10.0,10.0],des_obj_pose)
        return res.n_js, res.n_obj_pose
    '''
    def call(self,f_index,des_pts,obj_pose,des_obj_pose,js,T):
        f_=f_index
        rospy.wait_for_service('in_hand/primitive/in_grasp_manipulation')
        srv_fn=rospy.ServiceProxy('in_hand/primitive/in_grasp_manipulation',InGraspManipulationSd)
        res=srv_fn(js,obj_pose,des_pts,f_,T,[10.0,1.0,0.0],des_obj_pose)
        return res.n_js, res.n_obj_pose, res.joint_traj

def rigid_transform(c_pts,d_pts,obj_pose):
    curr_obj_pose=np.zeros(7)
    curr_obj_pose[0]=obj_pose.pose.position.x
    curr_obj_pose[1]=obj_pose.pose.position.y
    curr_obj_pose[2]=obj_pose.pose.position.z
    curr_obj_pose[3]=obj_pose.pose.orientation.x
    curr_obj_pose[4]=obj_pose.pose.orientation.y
    curr_obj_pose[5]=obj_pose.pose.orientation.z
    curr_obj_pose[6]=obj_pose.pose.orientation.w

    cpts=[]
    dpts=[]
    for i in range(len(c_pts)):
        cpts.append(np.array([c_pts[i].point.x, c_pts[i].point.y, c_pts[i].point.z]))
        dpts.append(np.array([d_pts[i].point.x, d_pts[i].point.y, d_pts[i].point.z]))
    frm=frames.transformMat()
    # Find transform from obj_pose to minimize error between c_pts and d_pts
    n_pose_mat=frm.rigid_svd(dpts,cpts,True)
    n_mat=np.matrix(frm.T_pose(curr_obj_pose))*np.matrix(n_pose_mat)
    
    n_pose=frm.pose_T(n_mat,False)
    #
    new_d_pose=n_pose-curr_obj_pose
    sca=0.08
    d_obj_pose=copy.deepcopy(obj_pose)
    d_obj_pose.pose.position.x=curr_obj_pose[0]+new_d_pose[0]*sca
    d_obj_pose.pose.position.y=curr_obj_pose[1]+new_d_pose[1]*sca
    d_obj_pose.pose.position.z=curr_obj_pose[2]+new_d_pose[2]*sca
    d_obj_pose.pose.orientation.x=n_pose[3]
    d_obj_pose.pose.orientation.y=n_pose[4]
    d_obj_pose.pose.orientation.z=n_pose[5]
    d_obj_pose.pose.orientation.w=n_pose[6]
    return d_obj_pose

def rigid_translate(c_pts,d_pts,obj_pose):
    curr_obj_pose=np.zeros(7)
    curr_obj_pose[0]=obj_pose.pose.position.x
    curr_obj_pose[1]=obj_pose.pose.position.y
    curr_obj_pose[2]=obj_pose.pose.position.z
    curr_obj_pose[3]=obj_pose.pose.orientation.x
    curr_obj_pose[4]=obj_pose.pose.orientation.y
    curr_obj_pose[5]=obj_pose.pose.orientation.z
    curr_obj_pose[6]=obj_pose.pose.orientation.w

    cpts=[]
    dpts=[]
    for i in range(len(c_pts)):
        cpts.append(np.array([c_pts[i].point.x, c_pts[i].point.y, c_pts[i].point.z]))
        dpts.append(np.array([d_pts[i].point.x, d_pts[i].point.y, d_pts[i].point.z]))
        
    #frm=frames.transformMat()
    # Find transform from obj_pose to minimize error between c_pts and d_pts
    #n_pose_mat=frm.rigid_svd(dpts,cpts,True)
    #n_mat=np.matrix(frm.T_pose(curr_obj_pose))*np.matrix(n_pose_mat)
    
    
    #
    delta_t=np.zeros(3)
    for i in range(len(dpts)):
        delta_t+=dpts[i]-cpts[i]
        
    new_d_pose=delta_t
    sca=0.08
    d_obj_pose=copy.deepcopy(obj_pose)
    d_obj_pose.pose.position.x=curr_obj_pose[0]-new_d_pose[0]*sca
    d_obj_pose.pose.position.y=curr_obj_pose[1]-new_d_pose[1]*sca
    d_obj_pose.pose.position.z=curr_obj_pose[2]-new_d_pose[2]*sca
    d_obj_pose.pose.orientation.x=curr_obj_pose[3]
    d_obj_pose.pose.orientation.y=curr_obj_pose[4]
    d_obj_pose.pose.orientation.z=curr_obj_pose[5]
    d_obj_pose.pose.orientation.w=curr_obj_pose[6]
    return d_obj_pose

def o_files(o_name,l):
    ss_names=[]
    for i in range(l):
        ss_names.append('r_'+o_name+'_'+str(i)+'.p')
    return ss_names

def get_gait_plan(o_name,file_name,prefix,l):

    # Read grasp data from file:
    file_ = open(file_name,'rb')
    object_file = pickle.load(file_)
    file_.close()

    obj_pose=object_file[0][0]
    
    grasp_loc='/home/bala/catkin_ws/src/grasp_gaits'
    o_mesh='file::///home/bala/object_models/'+o_name+'/google_16k/nontextured.stl'
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
    # Get object pose
    f_index=[0,1,2,3]
    js=object_file[0][1]
    n_js=js
    # Get allegro hand joint angles

    c_pts=object_file[0][3]

    
    #obj_pose.header.stamp=rospy.get_time()
    initial_obj_pose=copy.deepcopy(obj_pose)
    
    #f_index=[3,2,1,0]
    # Get initial hand joint configuration:
    hand_ik=handIKClient()

    time.sleep(1)



    #viz.viz_js(t_name="finger_gait",js=n_js)
    viz.viz_js(t_name="finger_gait",js=n_js)
    ## desired contact points:
    d_pts=object_file[-2][2]
    d_obj_pose=object_file[-2][0]
    viz.viz_obj_mesh(d_obj_pose,a=1.0,rgb=[1.0,1.0,0.0])
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
    # Visualize initial grasp:
    d_pts=n_d_pts
    viz.viz_obj_mesh(obj_pose,a=1.0,rgb=[1.0,1.0,0.0])
    viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])

    k_it=0
    thresh=1e-2
    relocate_client=relocateClient()
    in_grasp_sd_client=inGraspSdClient()
    t_name='planner/in_grasp_traj'
    traj_server=trajectoryServer(100,robot_name='allegro',topic_name=t_name,init_node=False)
    time.sleep(0.1)

    # Store initial grasp
    G_arr=[]
    G=[obj_pose,js,d_pts,c_pts]
    t0 = time.time()
    G_arr.append(G)
    P=[0,1,2,3]
    #P=[3,2,1,0]
    hand_contact=handContactClient()
    while(k_it<10):
        # Compute finger gait contact location
        for i in P:
            t_fg=time.time()
            f=i
            _,js,_,j_traj=relocate_client.call(obj_pose,c_pts,f,d_pts[f],js)
            t_end_fg=time.time()-t_fg
            viz.viz_js(t_name="finger_gait",js=js)
            G_arr.append([obj_pose,js,d_pts,c_pts,t_end_fg])


        _,c_pts=hand_contact.call(obj_pose,js,f_index,transform_points=False,initial_obj_pose=obj_pose)
        diff=np.zeros(4)
        for i in range(len(d_pts)):
            diff[i]=np.sqrt(((d_pts[i].point.x-c_pts[i].point.x)**2+
                     (d_pts[i].point.y-c_pts[i].point.y)**2+(d_pts[i].point.z-c_pts[i].point.z)**2))

        print diff
        if((diff < thresh).all()==True):
            print diff
            break
        d_obj_pose=rigid_transform(c_pts,d_pts,obj_pose)
        t_in_grasp=time.time()
        js,n_obj_pose,j_traj=in_grasp_sd_client.call(f_index,d_pts,obj_pose,d_obj_pose,js,T=10)
        t_end_in_grasp=time.time()-t_in_grasp

        # Publish joint traj:
            
        obj_mat=get_T_mat(n_obj_pose.pose)
        for i in range(len(d_pts)):
            new_pt=obj_mat*np.matrix(o_T_c[i])
            d_pts[i].point.x=new_pt[0,3]
            d_pts[i].point.y=new_pt[1,3]
            d_pts[i].point.z=new_pt[2,3]

        obj_pose=n_obj_pose


        viz.viz_obj_mesh(obj_pose,a=1.0,rgb=[1.0,1.0,0.0])
        viz.viz_js(t_name="finger_gait",js=js)
        #viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])
        G_arr.append([obj_pose,js,d_pts,c_pts,t_end_in_grasp])
        k_it+=1
        print k_it
    t1=time.time()
    total_=t1-t0
    print "TIME: "+str(total_)
    G_arr.append([k_it,total_,diff])

    out_file=prefix+'_'+o_name+'_'+str(l)+'.p'
    pickle.dump(G_arr, open( out_file, "wb" ) )

def o_files(o_name,l):
    ss_names=[]
    for i in range(l):
        ss_names.append('r_'+o_name+'_'+str(i)+'.p')
    return ss_names

if __name__=='__main__':
    #rospy.init_node('gait')
    o_name='pringles'
    file_list=o_files('pringles',5)
    obj_names=[o_name for i in range(5)]
    prefixes=['svd_r' for i in range(5)]
    idx=[0,1,2,3,4]
    for i in range(0,len(file_list)):
        get_gait_plan(obj_names[i],file_list[i],prefixes[i],idx[i])
        print i
