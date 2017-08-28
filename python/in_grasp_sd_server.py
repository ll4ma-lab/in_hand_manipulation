#!/usr/bin/env python
from cost_relaxed_rigid_gait import *
from kdl_allegro_model import *
import traj_opt as tjO
import time
import subprocess
import os
import signal
import sys
from in_grasp_manipulation.srv import InGraspManipulationSd, InGraspManipulationSdResponse
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from numpy.linalg import inv
#from multiprocessing import Pool
import numpy as np
delta_t=1.0/60.0

import PyKDL
from rospkg import RosPack
rp=RosPack()
rp.list()
path_g=rp.get_path('grasp_gaits')
sys.path.insert(0,path_g+'/scripts')
import grasp_class
import viz_markers
class Grasp:
    def __init__(self,joint_angles,obj_offset,contact_fingers=[0,1,3],palm_pose=np.zeros(6)):
        self.joint_angles=joint_angles
        self.contact_fingers=contact_fingers
        self.obj_offset=obj_offset

class inGraspPlanner:
    def __init__(self,f_meshes,viz=[]):
        self.robot_model=allegroKinematics(delta_t,fingers=[0])
        self.f_meshes=f_meshes
        self.viz=viz
        print "Created inGraspPlanner"

    def call(self,req):

        j0=np.array(req.init_joints.position)
        obj_pose=np.zeros(6)
        obj_pose[0]=req.object_pose.pose.position.x
        obj_pose[1]=req.object_pose.pose.position.y
        obj_pose[2]=req.object_pose.pose.position.z
        # convert quaternion to rpy
        R=PyKDL.Rotation()
        R=R.Quaternion(req.object_pose.pose.orientation.x, req.object_pose.pose.orientation.y, req.object_pose.pose.orientation.z, req.object_pose.pose.orientation.w)
        obj_pose[3],obj_pose[4],obj_pose[5]=R.GetRPY()
        des_pts=[]
        for i in range(len(req.des_contact_pts)):
            des_pts.append(np.array([req.des_contact_pts[i].point.x,
                            req.des_contact_pts[i].point.y,
                            req.des_contact_pts[i].point.z]))

        
        weights=[10.0,100.0,0.0,0.0,5000]
        f_index=[]
        for i in range(len(req.f_index)):
            f_index.append(req.f_index[i].data)
        T=req.T
        des_obj_pose=obj_pose

        self.update_grasp(obj_pose,j0.copy(),f_index,T)
        self.update_weights(weights,T)
        self.robot_model.f_meshes=self.f_meshes
        self.robot_model.viz=self.viz
        # Get contact point frames:
        cpts_mat=[]

        o_T_p=np.matrix(inv(self.robot_model.object_pose(j0,get_T_mat=True)))
        for i in range(len(des_pts)):
            p_T_c=np.eye(4)
            p_T_c[0:3,3]=des_pts[i]
            o_T_c=o_T_p*p_T_c
            cpts_mat.append(o_T_c)
        self.robot_model.cpts_mat=cpts_mat
        self.robot_model.cpts=des_pts
        u_arr,n_pose=self.plan(des_obj_pose,T)
        

        # joint trajectory :
        j_traj=JointTrajectory()
        j_traj.joint_names=['index_joint_0', 'index_joint_1', 'index_joint_2', 'index_joint_3', 'middle_joint_0', 'middle_joint_1', 'middle_joint_2', 'middle_joint_3', 'ring_joint_0', 'ring_joint_1', 'ring_joint_2', 'ring_joint_3', 'thumb_joint_0', 'thumb_joint_1', 'thumb_joint_2', 'thumb_joint_3']
        for i in range(len(u_arr)):
            jt_pt=JointTrajectoryPoint()
            jt_pt.positions=u_arr[i]
            j_traj.points.append(jt_pt)

        n_js=JointState()
        n_js.position=u_arr[-1]
        n_obj_pose=PoseStamped()
        n_obj_pose.pose.position.x=n_pose[0]
        n_obj_pose.pose.position.y=n_pose[1]
        n_obj_pose.pose.position.z=n_pose[2]
        R=PyKDL.Rotation()
        R=R.RPY(n_pose[3],n_pose[4],n_pose[5])
        x=R.GetQuaternion()
        n_obj_pose.pose.orientation.x=x[0]
        n_obj_pose.pose.orientation.y=x[1]
        n_obj_pose.pose.orientation.z=x[2]
        n_obj_pose.pose.orientation.w=x[3]
        return InGraspManipulationSdResponse(n_js,n_obj_pose,j_traj)
        
    def update_grasp(self,obj_pose,joints,f_index,T=10):
        # initial joint angles    

        self.robot_model=allegroKinematics(delta_t,T,fingers=f_index)
        self.robot_model.joints=joints.copy()
        # build obj frame:
        thumb_pose=self.robot_model.end_effector_pose(joints[3*4:3*4+4],3)

        t_T_O=np.eye(4)
        # Match initial rotation to palm fram
        R=PyKDL.Rotation()
        R=R.RPY(obj_pose[3],obj_pose[4],obj_pose[5])
        for k in range(3):
            for l in range(3):
                t_T_O[k,l]=R[k,l]
        t_T_O[0,3]=obj_pose[0]
        t_T_O[1,3]=obj_pose[1]
        t_T_O[2,3]=obj_pose[2]

        f_T_O=np.linalg.inv(thumb_pose)*t_T_O
        self.robot_model.object_frames=self.robot_model.update_object_frame(f_T_O,joints.copy(),finger=3)    
        self.robot_model.final_obj_dim_weights=np.array([100.0,100.0,100.0,-1.0,-1.0,1.0])

        self.robot_model.obj_dim_weights=np.array([100.0,100.0,100.0,1.0,1.0,-1.0])

    def update_weights(self,weights,T):
        self.robot_model.psi=[0.0,1.0,0.0]

        #print obj.weights
        self.robot_model.wp=weights[0]
        self.robot_model.wr=weights[1]/float(T)
        self.robot_model.wo=weights[2]/float(T)
        if(weights[3]==0.0):
            self.robot_model.wl=0.0
        else:
            self.robot_model.wl=weights[3]/float(T)
        self.max_iter=weights[4]

    
    def interpolate_joints(self,pos_array,H):
        t=len(pos_array)-1
        T_=H*10
        dt=T_/t
        new_traj=np.zeros((T_,16))
        for i in range(len(pos_array)-1):
            for k in range(len(pos_array[i])):
                new_traj[i*dt:i*dt+dt,k]=np.linspace(pos_array[i,k],pos_array[i+1,k],dt)
        
        return new_traj

    
    def plan(self,obj_des,H=10,psi=1):
        self.robot_model.T=H
        # Setup SNOPT:
        # Input to the optimization by the user
        u0=self.robot_model.joints.copy()
        joints=self.robot_model.joints.copy()
        max_iter=int(self.max_iter)
        # Initial pose of the object:
        x0=self.robot_model.object_pose(joints,3)


        
        x_des=obj_des
            
        # Storing desired object pose for faster solution:
        obj_mat=np.eye(4)
        R=PyKDL.Rotation.RPY(x_des[3],x_des[4],x_des[5])
        for k in range(3):
            for i in range(3):
                obj_mat[k,i]=R[k,i]
        obj_mat[0:3,3]=x_des[0:3]
        self.robot_model.des_finger_pose=self.robot_model.object_finger_diff(obj_mat,3)   

        # sampling object pose:
        u_input=np.array([joints.copy() for k in range(H+1)])
        
        
        
        x_d=np.array(list(np.array([x_des for k in range(H)])))

        robot_model=self.robot_model
        opt_options={'Iterations limit':max_iter,'Solution':'No','iPrint':0,'iSumm':0}
        opta=tjO.TrajOpt(u0,x_d,u_input,robot_model,cost,final_cost,cost_gradient,
                         final_cost_gradient,"SNOPT",opt_options)

        u_new,_=opta.optimize()  
    
        x_final=opta.robot.object_pose(u_new[-1])
 
        #joint_traj=self.interpolate_joints(u_new,H)

        return u_new,x_final
        

def main():
    rospy.init_node('in_grasp_manipulation_server')
    # load finger mesh
    # Get vertices of contact surface:
    robot_meshes=[path_g+'/data/robot_models/cvx_index.ply',
                  path_g+'/data/robot_models/poisson_middle.ply',
                  path_g+'/data/robot_models/poisson_ring.ply',
                  path_g+'/data/robot_models/poisson_thumb.ply']

    f_meshes=['file://'+path_g+'/data/robot_models/cvx_index.ply',
              'file://'+path_g+'/data/robot_models/poisson_middle.ply',
              'file://'+path_g+'/data/robot_models/poisson_ring.ply',
              'file://'+path_g+'/data/robot_models/poisson_thumb.ply']

    obj_mesh=path_g+'/data/object_models/box.stl'
    # import object mesh, robot meshes
    grasp_g=grasp_class.grasp([0,1,2,3],robot_meshes,obj_mesh)
    # visualize f_mesh
    viz=viz_markers.VizInHand(finger_meshes=f_meshes,init_node=False)
    pose=np.zeros(7)
    pose[-1]=1.0
    viz.viz_reachability(pose,f_meshes,a=0.5)
    in_grasp=inGraspPlanner(grasp_g.finger_meshes)
    rospy.Service('in_hand/primitive/in_grasp_manipulation',InGraspManipulationSd,in_grasp.call)
    rospy.spin()
    
main()
