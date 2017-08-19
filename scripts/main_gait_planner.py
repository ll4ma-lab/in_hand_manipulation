#!/usr/bin/env python
import rospy
from in_hand_manipulation.srv import GetHandIK,GetHandContact
from grasp_gaits.srv import GetEllipse,RelocateFinger
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped,PointStamped
import numpy as np
from viz_markers import VizInHand

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

    def call(self,obj_pose,js,f_index=[0,1,2,3]):
        f_=[]
        for i in range(len(f_index)):
            idx=Int64()
            idx.data=f_index[i]
            f_.append(idx)
        rospy.wait_for_service('/in_hand_manipulation/get_hand_contacts')
        srv_fn=rospy.ServiceProxy('/in_hand_manipulation/get_hand_contacts',GetHandContact)
        res=srv_fn(obj_pose,js,f_)
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
    obj_pose.pose.position.x=0.08
    obj_pose.pose.position.y=-0.08
    obj_pose.pose.position.z=0.015
    obj_pose.pose.orientation.x=-0.706825
    obj_pose.pose.orientation.y=0.0
    obj_pose.pose.orientation.z=0.0
    obj_pose.pose.orientation.w=0.707


    # Get allegro hand joint angles
    js=JointState()
    js.position=np.ones(16)*0.3
    js.position[1]=0.5
    
    # Get initial contact points:
    hand_contact=handContactClient()
    _,c_pts=hand_contact.call(obj_pose,js,f_index=[0,1,2,3])

    # Visulize initial grasp:
    viz.viz_obj_mesh(obj_pose)
    viz.viz_cpts([c_pts[0]],t='cpt',rgb=[1.0,0.0,0.0])
    
    # Desired contact points:
    obj_pose.pose.position.y-=0.03
    obj_pose.pose.position.x+=0.03
    _,d_pts=hand_contact.call(obj_pose,js,f_index=[0,1,2,3])
    obj_pose.pose.position.y+=0.03
    obj_pose.pose.position.x-=0.03
    viz.viz_cpts([d_pts[0]],t='d_cpt',rgb=[0.0,1.0,0.0])    


    # Single step opt testing:
    # Get ellipses:
    contact_client=ellipseClient()
    ellipses=contact_client.call(obj_pose,f_index=[0])
    print "got ellipses!"

    # Compute finger gait contact location
    f=0
    relocate_client=relocateClient()
    n_cpt,_=relocate_client.call(obj_pose,c_pts,f,d_pts[f],ellipses[f])
    viz.viz_cpts([n_cpt],t='n_cpt',rgb=[0.0,0.0,1.0])    
    #print n_cpt.point
    #print c_pts[f].point
    
    # Update hand joint configuration
    hand_ik=handIKClient()
    n_js=hand_ik.call([f],[n_cpt],js)

    #print n_js
    # In-grasp to expand workspace
