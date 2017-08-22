#!/usr/bin/env python
import rospy
from in_hand_manipulation.srv import GetHandIK,GetHandContact
from grasp_gaits.srv import GetEllipse,RelocateFinger,RelocateObj
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped,PointStamped
import numpy as np
from viz_markers import VizInHand
import copy

import pickle

class ellipseClient:
    def __init__(self):
        print "Yay! initialized ellipse client"
    def call(self,obj_pose,f_index=[0]):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)

        rospy.wait_for_service('/regrasp_planner/update_contact_ellipse')
        get_ellipse=rospy.ServiceProxy('/regrasp_planner/update_contact_ellipse',GetEllipse)
        res=get_ellipse(obj_pose,f_)
        
        ellipses=res.ellipses
        return ellipses
        
        
    def tester(self,f_index=[0]):
        p=PoseStamped()
        p.pose.position.x=0.08
        p.pose.position.y=-0.08
        p.pose.position.z=0.015
        p.pose.orientation.x=-0.706825
        p.pose.orientation.y=0.0
        p.pose.orientation.z=0.0
        p.pose.orientation.w=0.707
        
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)

        rospy.wait_for_service('/regrasp_planner/update_contact_ellipse')
        get_ellipse=rospy.ServiceProxy('/regrasp_planner/update_contact_ellipse',GetEllipse)
        res=get_ellipse(p,f_)
        return res.ellipses

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
    def call(self,obj_pose,c_pts,f_,des_cpt,ellipse):
        rospy.wait_for_service('/regrasp_planner/relocate_finger')
        srv_fn=rospy.ServiceProxy('/regrasp_planner/relocate_finger',RelocateFinger)
        res=srv_fn(obj_pose,c_pts,f_,des_cpt,ellipse)
        return res.next_contact_pt,res.stability_score

class relocateObjClient:
    def __init__(self):
        print "Relocate Object client: Ready!!"

    def call(self,f_index,ellipses,obj_pose,cpts,des_cpts):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)
        rospy.wait_for_service('/regrasp_planner/relocate_object')
        srv_fn=rospy.ServiceProxy('/regrasp_planner/relocate_object',RelocateObj)
        res=srv_fn(f_,ellipses,obj_pose,cpts,des_cpts)
        return res.n_obj_pose

if __name__=='__main__':
    grasp_loc='/home/bala/catkin_ws/src/grasp_gaits'
    o_mesh='file://'+grasp_loc+'/data/object_models/box_bin.stl'
    cs_meshes=['file://'+grasp_loc+'/scripts/f0.stl',
              'file://'+grasp_loc+'/scripts/f1.stl',
              'file://'+grasp_loc+'/scripts/f2.stl',
              'file://'+grasp_loc+'/scripts/f3.stl']
    
    f_meshes=['file://'+grasp_loc+'/data/robot_models/poisson_index.ply',
              'file://'+grasp_loc+'/data/robot_models/poisson_middle.ply',
              'file://'+grasp_loc+'/data/robot_models/poisson_ring.ply',
              'file://'+grasp_loc+'/data/robot_models/poisson_thumb.ply']


    viz=VizInHand(o_mesh,f_meshes)    
    

    ## Initial grasp:
    # Get object pose
    obj_pose=PoseStamped()
    obj_pose.pose.position.x=0.0
    obj_pose.pose.position.y=-0.14
    obj_pose.pose.position.z=0.012
    obj_pose.pose.orientation.x=-0.706825
    obj_pose.pose.orientation.y=0.0
    obj_pose.pose.orientation.z=0.0
    obj_pose.pose.orientation.w=0.707


    # Get allegro hand joint angles
    js=JointState()
    js.position=np.ones(16)*0.3
    #js.position[1]=0.5
    js.position[12]=1.4
    js.position[14]=0.8
    # Get initial contact points:
    hand_contact=handContactClient()
    _,c_pts=hand_contact.call(obj_pose,js,f_index=[0,1,2,3],transform_points=False,initial_obj_pose=obj_pose)
    initial_obj_pose=copy.deepcopy(obj_pose)
    # Desired contact points:
    obj_pose.pose.position.y-=0.10
    obj_pose.pose.position.x-=0.02
    js.position[12]=1.55
    js.position[15]=0.8
    js.position[14]=0.8
    _,d_pts=hand_contact.call(obj_pose, js, f_index=[0,1,2,3], transform_points=True, initial_obj_pose=initial_obj_pose)
    #viz.viz_obj_mesh(obj_pose)
    obj_pose.pose.position.y+=0.10
    obj_pose.pose.position.x+=0.02
        
    #for i in range(3):
    #    d_pts[i]=c_pts[i]
            
    k=0
    thresh=0.006
    
    while(k<10):
        
        # Visulize initial grasp:
        viz.viz_obj_mesh(obj_pose,a=1.0)
        viz.viz_cpts(c_pts,t='cpt',rgb=[1.0,0.0,0.0])
        viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])    
     
        # Single step opt testing:
        # Get ellipses:
        f_index=[0,1,2,3]
        contact_client=ellipseClient()
        
        ellipses=contact_client.call(obj_pose,f_index)
        #pickle.dump(ellipses,open( "ellipses.p", "wb" ))
        #ellipses=pickle.load(open( "ellipses.p", "rb" ))
        print "got ellipses!"

        # Compute finger gait contact location
        relocate_client=relocateClient()
        for i in f_index:
            f=i
            n_cpt,_=relocate_client.call(obj_pose,c_pts,f,d_pts[f],ellipses[f])
            c_pts[f]=n_cpt
            viz.viz_cpts([n_cpt],t='n_cpt',rgb=[0.0,0.0,1.0])
        viz.viz_cpts(c_pts,t='cpt',rgb=[1.0,0.0,0.0])
        print "Relocated fingers"

        '''
        # Update hand joint configuration
        hand_ik=handIKClient()
        n_js=hand_ik.call([f],[n_cpt],js)
        '''
        diff=np.zeros(4)
        for i in range(len(d_pts)):            diff[i]=np.abs(d_pts[i].point.x-c_pts[i].point.x)+np.abs(d_pts[i].point.y-c_pts[i].point.y)+np.abs(d_pts[i].point.z-c_pts[i].point.z)

        print diff
        if((diff < thresh).all()==True):
            break

    

        # Relocate object to move workspace:
        # inputs: f_index,contact points, ellipses, object pose
        # outputs: object_pose
        print obj_pose
        f_index=[0,1,2,3]
        relocate_obj=relocateObjClient()
        n_obj_pose=relocate_obj.call(f_index,ellipses,obj_pose,c_pts,d_pts)

    
        viz.viz_obj_mesh(n_obj_pose,t='next_obj',rgb=[1.0,0.0,0.0])
        diff=np.zeros(4)
        for i in range(len(d_pts)):
            d_pts[i].point.x+=n_obj_pose.pose.position.x-obj_pose.pose.position.x
            d_pts[i].point.y+=n_obj_pose.pose.position.y-obj_pose.pose.position.y
            d_pts[i].point.z+=n_obj_pose.pose.position.z-obj_pose.pose.position.z
            c_pts[i].point.x+=n_obj_pose.pose.position.x-obj_pose.pose.position.x
            c_pts[i].point.y+=n_obj_pose.pose.position.y-obj_pose.pose.position.y
            c_pts[i].point.z+=n_obj_pose.pose.position.z-obj_pose.pose.position.z
            diff[i]=np.abs(d_pts[i].point.x-c_pts[i].point.x)+np.abs(d_pts[i].point.y-c_pts[i].point.y)+np.abs(d_pts[i].point.z-c_pts[i].point.z)
            

        print diff
        if((diff < thresh).all()==True):
            break
        
        obj_pose=n_obj_pose
        # transform d_pts:

        
        k+=1

    # In-grasp to expand workspace
    # inputs: joint_angles, f_index, ellipses, desired contact points

    
    # Output: joint_angles

    # Repeat from get contact points
