#!/usr/bin/env python
from cost_relaxed_rigid_obj_sampled import *
from kdl_allegro_model import *
import traj_opt as tjO
import time
import subprocess
import os
import signal
import sys

#from multiprocessing import Pool
import numpy as np
from object_list import *
delta_t=1.0/60.0
T=5
import robot_node as robot

class Grasp:
    def __init__(self,joint_angles,obj_offset,contact_fingers=[0,1,3],palm_pose=np.zeros(6)):
        self.joint_angles=joint_angles
        self.contact_fingers=contact_fingers
        self.obj_offset=obj_offset

class inGraspPlanner:
    def __init__(self):
        print "Created inGraspPlanner"

    def update_grasp(self,grasp,finger_object_frame):
        self.grasp=grasp
        # Reinitialize robot_model variables:
        #robot_model
        #D=grasp.obj_offset # Distance of the object from finger  

        # initial joint angles    
        joints=grasp.joint_angles
        #obj_name=obj.name

        robot_model=allegroKinematics(delta_t,T,fingers=grasp.contact_fingers)
        robot_model.joints=joints
        #robot_model.object_frames=robot_model.add_object_frame(D,joints,finger=3)
        
        robot_model.object_frames=robot_model.update_object_frame(finger_object_frame,joints,finger=3)    
        robot_model.final_obj_dim_weights=np.array([100.0,100.0,100.0,-1.0,-1.0,1.0])

        robot_model.obj_dim_weights=np.array([100.0,100.0,100.0,1.0,1.0,-1.0])


        self.robot_model=robot_model
    def update_weights(self,obj):
        self.robot_model.psi=[0.0,1.0,0.0]

        #print obj.weights
        self.robot_model.wp=obj.weights[0]
        self.robot_model.wr=obj.weights[1]/float(T)
        self.robot_model.wo=obj.weights[2]/float(T)
        if(obj.weights[3]==0.0):
            self.robot_model.wl=0.0
        else:
            self.robot_model.wl=obj.weights[3]/float(T)
        self.max_iter=obj.weights[4]
        #print obj.weights
    
    def interpolate_joints(self,pos_array):
        t=len(pos_array)-1
        T_=100
        dt=T_/t
        new_traj=np.zeros((T_,16))

        for i in range(len(pos_array)-1):
            for k in range(len(pos_array[i])):
                new_traj[i*dt:i*dt+dt,k]=np.linspace(pos_array[i,k],pos_array[i+1,k],dt)
        return new_traj

    def update_pre_plan(self,interpolated_plan):
        pre_plan=np.zeros((10,16))
        for i in range(len(interpolated_plan)):
            if(i%10==0):
                pre_plan[i/10]=interpolated_plan[i]
                self.pre_plan=pre_plan[0:T]
    
    def plan(self,obj_des):
        #self.robot_model.T=T
        # Setup SNOPT:
        # Input to the optimization by the user
        u0=self.robot_model.joints.copy()
        joints=self.robot_model.joints.copy()
        max_iter=int(self.max_iter)
        # Initial pose of the object:
        x0=self.robot_model.object_pose(joints,3)
        #print x0
        
        
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
        for i in range(T):
            sample_pose=x0+(x_des-x0)*float(i)/T
            # Building transformation matrix:
            x_des_mat=np.eye(4)
            x_des_mat[0:3,3]=sample_pose[0:3]
            R=PyKDL.Rotation.RPY(sample_pose[3],sample_pose[4],sample_pose[5])
            for i in range(3):
                for j in range(3):
                    x_des_mat[i,j]=R[i,j]
            linear_poses.append(list(self.robot_model.object_finger_diff(x_des_mat,3)))

        

        self.robot_model.linear_finger_poses=linear_poses
        u_input=np.array([joints.copy() for k in range(T+1)]) 
        u_input[1:]=self.pre_plan
        #u_input[0]=joints.copy()
        x_d=np.array(list(np.array([x_des for k in range(T)])))

        robot_model=self.robot_model
        opt_options={'Iterations limit':max_iter,'Solution':'No','iPrint':0,'iSumm':0}
        opta=tjO.TrajOpt(u0,x_d,u_input,robot_model,cost,final_cost,cost_gradient,
                         final_cost_gradient,"SNOPT",opt_options)

        u_new,_=opta.optimize()  
    
        x_final=opta.robot.object_pose(u_new[-1])
        self.pre_plan=u_new[1:]

        
        joint_traj=self.interpolate_joints(u_new)

        print x_des
        print x_final
        
        return joint_traj
        

def main():
    allegroStates=robot.robotStates(listen_aruco=True)   
    method='relaxed-rigidity'
    f_name='../data/trajectories/exp_2_goal_poses.csv'
    objects=ObjectList(f_name,'')
    objects.get_weights('../data/weights/feedback_weights.csv')
    objects.add_aruco_centroid_locations('../data/initial_data/object_grasps/object_centroids.csv')

    # Initializing trajectory with previous run:
    traj_name='../data/trajectories/debug/Lego_0_30'
    pre_plan=np.loadtxt(open(traj_name,'r'))
    
    t_no=30#int(raw_input("Enter trajectory number: "))
    obj=objects.arr[t_no]
    print obj.name
    print obj.aruco

    # Move robot to initial grasp pose:
    allegroStates.position_mode()
    allegroStates.publish_grasp(obj.grasp)

    planner=inGraspPlanner()
    planner.update_pre_plan(pre_plan)
    raw_input("Place object")
    # publish object pose by offsetting from aruco marker:
    P_=[] # list of subprocesses
    P_.append(allegroStates.tf_pose_pub_process(obj.aruco,'obj_ar','ar') )
    P_.append(allegroStates.tf_pose_pub_process(obj.des,'G'))
    P_.append(allegroStates.tf_pose_pub_process(obj.initial_pose,'i') )
                
    raw_input("Confirm object tracking")
    replan=1
    while(replan==1):
        # Update initial grasp
        # Get new robot state:
        initial_robot_joints=allegroStates.joint_states.position
        # Get finger object frame:
        obj_grasp=Grasp(np.array(initial_robot_joints),obj.offset,obj.fing)
        f_T_o=allegroStates.tf_find_T_mat('obj_ar','thumb_tip')
        planner.update_grasp(obj_grasp,f_T_o)
        planner.update_weights(obj)
        traj=planner.plan(obj.des)
        raw_input("Enter to run traj:")
        # Run traj:
        allegroStates.publish_joint_trajectory(traj[0:10])
        replan=int(raw_input("Replan with new pose: "))

    for i in P_:
        i.send_signal(subprocess.signal.SIGINT)

main()
