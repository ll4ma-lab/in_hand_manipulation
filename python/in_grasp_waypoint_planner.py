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
T=10
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
        #print finger_object_frame
        # initial joint angles    
        joints=grasp.joint_angles
        #obj_name=obj.name

        self.robot_model=allegroKinematics(delta_t,T,fingers=grasp.contact_fingers)
        self.robot_model.joints=joints.copy()
        #robot_model.object_frames=robot_model.add_object_frame(D,joints,finger=3)
        
        self.robot_model.object_frames=self.robot_model.update_object_frame(finger_object_frame,joints,finger=3)    
        self.robot_model.final_obj_dim_weights=np.array([100.0,100.0,100.0,-1.0,-1.0,1.0])

        self.robot_model.obj_dim_weights=np.array([100.0,100.0,100.0,1.0,1.0,-1.0])

    def update_weights(self,obj):
        self.robot_model.psi=[0.0,1.0,0.0]

        #print obj.weights
        self.robot_model.wp=obj.weights[0]/float(T)
        self.robot_model.wr=obj.weights[1]/float(T)
        self.robot_model.wo=obj.weights[2]/float(T)
        self.robot_model.wl=0.0
        self.max_iter=obj.weights[4]
        #print obj.weights
    
    def interpolate_joints(self,pos_array,H):
        t=len(pos_array)-1
        T_=H*10
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
    
    def plan(self,obj_des,H=10,psi=1):
        self.robot_model.T=H
        # Setup SNOPT:
        # Input to the optimization by the user
        u0=self.robot_model.joints.copy()
        joints=self.robot_model.joints.copy()
        max_iter=int(self.max_iter)
        # Initial pose of the object:
        x0=self.robot_model.object_pose(joints,3)
        #print x0
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
        
        u_input[1:]=self.pre_plan[:H-psi]
        if(H<=4):
            u_input=np.array([joints.copy() for k in range(H+1)])

        #u_input[0]=joints.copy()
        x_d=np.array(list(np.array([x_des for k in range(H)])))

        robot_model=self.robot_model
        opt_options={'Iterations limit':max_iter,'Solution':'No','iPrint':0,'iSumm':0}
        opta=tjO.TrajOpt(u0,x_d,u_input,robot_model,cost,final_cost,cost_gradient,
                         final_cost_gradient,"SNOPT",opt_options)

        u_new,_=opta.optimize()  
    
        x_final=opta.robot.object_pose(u_new[-1])
        self.pre_plan=u_new[psi:]

        joint_traj=u_new#self.interpolate_joints(u_new,H)

        #print x_des
        #print x_final
        
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
    #traj_name='../data/trajectories/debug/Apple_0_0'
    pre_plan=np.load('new_plan.npy')#np.loadtxt(open(traj_name,'r'))
    
    t_no=30#int(raw_input("Enter trajectory number: "))
    obj=objects.arr[t_no]
    print obj.name
    
    x0=obj.initial_pose
    x_des=obj.des
    linear_poses=[]
    
    for i in range(1,T):
        sample_pose=x0+(x_des-x0)*float(i)/T
        # Building transformation matrix:
        x_des_mat=np.eye(4)
        x_des_mat[0:3,3]=sample_pose[0:3]
        linear_poses.append(sample_pose)

    # Update goal pose:

    # Move robot to initial grasp pose:
    allegroStates.position_mode()
    allegroStates.publish_grasp(obj.grasp)
    robot_model=allegroKinematics(delta_t,T,fingers=obj.fing)
    print obj.grasp



    planner=inGraspPlanner()
    planner.update_pre_plan(pre_plan)
    raw_input("Place object")
    # publish object pose by offsetting from aruco marker:
    P_=[] # list of subprocesses
    P_.append(allegroStates.tf_pose_pub_process(obj.aruco,'obj_ar','ar') )
    P_.append(allegroStates.tf_pose_pub_process(obj.des,'G'))
    P_.append(allegroStates.tf_pose_pub_process(obj.initial_pose,'i') )
    allegroStates.loop_rate.sleep()
    #initial_frame=allegroStates.tf_find_transform('/i','/obj_ar')
    #palm_rotation=allegroStates.tf_find_transform('/palm_link','/obj_ar')
    #raw_input('remove initial error')
    #P_.append(allegroStates.tf_pose_pub_process(initial_frame,'/obj','/obj_ar'))
                
    raw_input("Confirm object tracking")
    replan=1
    H=8
    k=0
    psi=7
    vel_gains=[6.0,0.0,6.0]

    # update goal pose:
    waypoint=8
    obj.des=linear_poses[waypoint]

    way_tf=allegroStates.tf_pose_pub_process(obj.des,'W')
    while(replan==1):
        # Update initial grasp
        # Get new robot state:
        initial_robot_joints=np.array(allegroStates.joint_states.position)
        vel_offset=np.zeros((4,4))
        tips=['index_tip','middle_tip','ring_tip','thumb_tip']
        fing_T_O=np.zeros((4,3))
        for i in obj.fing:
            vel_offset[i,0:3]=vel_gains*allegroStates.tf_find_transform(tips[i],'/obj_ar')[0:3]
            fing_T_O[i]=allegroStates.tf_find_transform(tips[i],'/obj_ar')[0:3]
        p_T_o=allegroStates.tf_find_T_mat('obj_ar','palm_link')
        new_grasp=allegroStates.tighten_grasp(initial_robot_joints.copy(),robot_model,p_T_o,vel_offset)
        
        allegroStates.publish_grasp(new_grasp)

        # Get finger object frame:
        new_offset=allegroStates.tf_find_transform('thumb_tip','/obj_ar')
        #print new_offset
        #print obj.offset
        obj_grasp=Grasp(initial_robot_joints.copy(),obj.offset,obj.fing)
        f_T_o=allegroStates.tf_find_T_mat('obj_ar','thumb_tip')
        planner.update_grasp(obj_grasp,f_T_o)
        planner.update_weights(obj)
        #sta_t = time.time()
        
        traj=planner.plan(obj.des,H,psi)

        #elap=time.time()-sta_t
        #print "Time to compute(seconds): "+str(elap)
        #print "Time horizon: "+str(H)
        tight_traj=allegroStates.tighten_traj(np.array(traj[0:psi*10]).copy(),robot_model,vel_gains,f_T_o,fing_T_O)
        #if(k==0):
        raw_input('run traj')
      
        allegroStates.publish_joint_trajectory(tight_traj)
       
        #break
        # check error
        curr_obj_pose=allegroStates.tf_find_transform('obj_ar')
        err=np.linalg.norm(curr_obj_pose[0:3]-obj.des[0:3])

        if(err<0.009):
            waypoint+=2
            if(waypoint<len(linear_poses)):
                print 'Reached waypoint'
                way_tf.send_signal(subprocess.signal.SIGINT)
                obj.des=linear_poses[waypoint]
                way_tf=allegroStates.tf_pose_pub_process(obj.des,'W')
                replan=1
            else:
                replan=2
        #H=H-psi
        if(k>50):
            replan=2
    for i in P_:
        i.send_signal(subprocess.signal.SIGINT)

main()
