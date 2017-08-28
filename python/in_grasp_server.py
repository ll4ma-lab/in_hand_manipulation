#!/usr/bin/env python
from cost_relaxed_rigid_obj_sampled import *
from kdl_allegro_model import *
import traj_opt as tjO
import time
import subprocess
import os
import signal
import sys
from in_grasp_manipulation.srv import InGraspManipulation, InGraspManipulationResponse
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
#from multiprocessing import Pool
import numpy as np
delta_t=1.0/60.0

import PyKDL

class Grasp:
    def __init__(self,joint_angles,obj_offset,contact_fingers=[0,1,3],palm_pose=np.zeros(6)):
        self.joint_angles=joint_angles
        self.contact_fingers=contact_fingers
        self.obj_offset=obj_offset

class inGraspPlanner:
    def __init__(self):
        self.robot_model=allegroKinematics(delta_t,fingers=[0])
        print "Created inGraspPlanner"

    def call(self,req):
        #
        j0=np.array(req.init_joints.position)
        obj_pose=np.zeros(6)
        obj_pose[0]=req.object_pose.position.x
        obj_pose[1]=req.object_pose.position.y
        obj_pose[2]=req.object_pose.position.z
        # convert quaternion to rpy
        R=PyKDL.Rotation()
        R=R.Quaternion(req.object_pose.orientation.x, req.object_pose.orientation.y, req.object_pose.orientation.z, req.object_pose.orientation.w)
        obj_pose[3],obj_pose[4],obj_pose[5]=R.GetRPY()
        des_obj_pose=np.zeros(6)
        des_obj_pose[0]=req.desired_object_pose.position.x
        des_obj_pose[1]=req.desired_object_pose.position.y
        des_obj_pose[2]=req.desired_object_pose.position.z
        R_=PyKDL.Rotation()
        R_=R_.Quaternion(req.desired_object_pose.orientation.x, req.desired_object_pose.orientation.y, req.desired_object_pose.orientation.z, req.desired_object_pose.orientation.w)
        des_obj_pose[3],des_obj_pose[4],des_obj_pose[5]=R_.GetRPY()

        weights=[10.0,1.0,0.01,0.0,2000]
        f_index=req.fingers
        r_f=req.rigid_finger
        T=req.T
    

        self.update_grasp(obj_pose,j0,f_index,T)
        self.update_weights(weights,T)

        u_arr,n_pose=self.plan(des_obj_pose,T)
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


        # joint trajectory :
        j_traj=JointTrajectory()
        j_traj.joint_names=['index_joint_0', 'index_joint_1', 'index_joint_2', 'index_joint_3', 'middle_joint_0', 'middle_joint_1', 'middle_joint_2', 'middle_joint_3', 'ring_joint_0', 'ring_joint_1', 'ring_joint_2', 'ring_joint_3', 'thumb_joint_0', 'thumb_joint_1', 'thumb_joint_2', 'thumb_joint_3']
        for i in range(len(u_arr)):
            jt_pt=JointTrajectoryPoint()
            jt_pt.positions=u_arr[i]
            j_traj.points.append(jt_pt)
        n_js=JointState()
        n_js.position=u_arr[-1]

        return InGraspManipulationResponse(n_js,n_obj_pose,j_traj)
        
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
        x0=self.robot_model.object_pose(joints.copy(),3)
        #print x0
        print x0

        
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
        linear_poses=[]
        for i in range(H):
            sample_pose=x0+(x_des-x0)*float(i)/H
            # Building transformation matrix:
            x_des_mat=np.eye(4)
            x_des_mat[0:3,3]=sample_pose[0:3]
            R=PyKDL.Rotation.RPY(sample_pose[3],sample_pose[4],sample_pose[5])
            for i in range(3):
                for j in range(3):
                    x_des_mat[i,j]=R[i,j]
            linear_poses.append(list(self.robot_model.object_finger_diff(x_des_mat,3)))

        

        self.robot_model.linear_finger_poses=linear_poses
        
        u_input=np.array([joints.copy() for k in range(H+1)])
        
        
        
        x_d=np.array(list(np.array([x_des for k in range(H)])))

        robot_model=self.robot_model
        opt_options={'Iterations limit':max_iter,'Solution':'No','iPrint':0,'iSumm':0}
        opta=tjO.TrajOpt(u0,x_d,u_input,robot_model,cost,final_cost,cost_gradient,
                         final_cost_gradient,"SNOPT",opt_options)


        u_new,_=opta.optimize()  
        final_cost(x_des,u_new[-1],robot_model)
        final_cost(x_des,u0,robot_model)
        x_final=opta.robot.object_pose(u_new[-1])
 
        joint_traj=u_new#self.interpolate_joints(u_new,H)
        
        #print x_des
        #print x_final

        return joint_traj,x_final
        

def main():
    rospy.init_node('in_grasp_manipulation_server')

    in_grasp=inGraspPlanner()
    rospy.Service('in_hand/primitive/in_grasp_manipulation',InGraspManipulation,in_grasp.call)
    rospy.spin()
    
main()
