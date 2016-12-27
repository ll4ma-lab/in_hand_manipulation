# Gets list of trajectories and picks 5 best trials based on errors for each trajectory

import csv
import numpy as np
import PyKDL
import copy
from kdl_allegro_model import *
import time
from multiprocessing import Pool

from object_list import *


class objectClass:
    def __init__(self,name):
        self.traj_no=[0,1,2,3,4]
        self.name=name      
        self.traj=[]
        for i in self.traj_no:
            self.traj.append(trajClass())

class trial:
    def __init__(self):
        self.reached=[]
        self.position_err=0.0
        self.orient_err=0.0
        self.norm_position_err=0.0
        self.norm_orientation_err=0.0

class trajClass:
    def __init__(self):
        self.trials=[]
        self.desired=np.zeros(6)
        self.length_position=-1.0
        #self.tj_no=tj_no


def angle_axis_from_quat(q):
    if(q[3]>1.0):
        q_n=np.linalg.norm(q)
        q=q/q_n
    angle=2*(np.arccos(q[3]))
    return angle
def axis_from_quat(q):
    angle=2*(np.arccos(q[3]))
    if angle !=0:
        x=q[0]/np.sqrt(1-q[3]*q[3])
        y=q[1]/np.sqrt(1-q[3]*q[3])
        z=q[2]/np.sqrt(1-q[3]*q[3])
    else:
        x=0.0
        y=0.0
        z=0.0
    axis=[x,y,z]
    return np.array(axis)

def compute_traj_length():
    file="ral_same_poses.csv"
    folder_to_write="ral_lin/"
    obj_list=ObjectReachList(file,folder_to_write)  
    sc_pool=Pool()
    sta_t = time.time()
    tr=0

    for res in sc_pool.imap_unordered(multi_obj,obj_list.arr,chunksize=1):
        row=res
        q=row[-1]
        obj_list.arr[q].sol=row[0]
        #position_length[q]=row[0][0]
        #print obj_list.arr[q].sol
        tr+=1
    return obj_list


def multi_obj(obj):
    delta_t=1.0/60
    T=10
    i_number=obj.row
    
    # Input to the optimization by the user
    D=obj.offset # Distance of the object from finger  

    # initial joint angles    
    joints=obj.grasp
    obj_name=obj.name

    robot_model=allegroKinematics(delta_t,T,fingers=obj.fing)
    robot_model.joints=joints
    robot_model.object_frames=robot_model.add_object_frame(D,joints,finger=3)    

    robot_model.obj_dim_weights=np.array([1.0,1.0,1.0,-1.0,-1.0,1.0])       
        
    robot_model.wp=1000.0
    robot_model.wr=20.0
    robot_model.wo=0.01
    u0=joints.copy()


    # Initial pose of the object:
    x0=robot_model.object_pose(joints,3)

    joints_des=obj.des
    #Read joint trajectory from file:
        

    f_name=obj.f_name
    #folder=f_name
    f=open(f_name,'r')
    joint_traj=np.loadtxt(f)

    # Computing length of trajectory:
    position_length=0.0
    orient_length=0.0
    prev_pos=x0
    for i in range(1,len(joint_traj)):
        obj_pose=robot_model.object_pose(joint_traj[i],3)
        position_length+=np.linalg.norm(obj_pose[0:3]-prev_pos[0:3])
        old_quat=np.array(PyKDL.Rotation.RPY(prev_pos[3],prev_pos[4],prev_pos[5]).GetQuaternion())
        new_quat=np.array(PyKDL.Rotation.RPY(obj_pose[3],obj_pose[4],obj_pose[5]).GetQuaternion())
        q_sum=np.linalg.norm(new_quat+old_quat,ord=2)
        q_diff=np.linalg.norm(new_quat-old_quat,ord=2)
        R_err=np.min([q_sum,q_diff])
        orient_length+=R_err
        prev_pos=obj_pose
    return [position_length,orient_length],i_number


        
def file_import(f):
    
    trial_list=[]
    empty=0
    with open(f,'r') as csvfile:
        csvreader=csv.reader(csvfile,delimiter=',',quotechar='"')
        line=1
        new_obj=objectClass("temp")
        obj_list=[]
        init_flag=False
        # Create object class
        for row in csvreader:
            traj_n=int(row[1])-1
            # store under name,trajectory
            if(new_obj.name != row[0]):
                if init_flag:
                    obj_list.append(new_obj)
                else:
                    init_flag=True

                new_obj=objectClass(row[0])
                # Compute trajectory lengths
            des_pose=np.array(map(float,row[2:8]))


            if not(row[8] in (None, "")):

                reached_pose=np.array(map(float,row[8:14]))
                new_trial=trial()
                
                
                new_trial.reached.append(reached_pose)    
                # Compute errors       
                new_obj.traj[traj_n].desired=des_pose
                # Compute error between reached and desired
                R_des=PyKDL.Rotation.RPY(des_pose[3],des_pose[4],des_pose[5])
                R_reach=PyKDL.Rotation.RPY(reached_pose[3],reached_pose[4],reached_pose[5])

            
                des=np.array(R_des.GetQuaternion())
                reach=np.array(R_reach.GetQuaternion())
                q_sum=np.linalg.norm(des+reach,ord=2)
                q_diff=np.linalg.norm(des-reach,ord=2)
                q_err=np.min([q_sum,q_diff])
                # Computing angle axis error:
                q_sum=des+reach
                q_diff=des-reach

                R_1=abs(angle_axis_from_quat(q_sum))
                R_2=abs(angle_axis_from_quat(q_diff))
                angle_err=np.min([R_1,R_2])


                #position error:
                pos_err=np.linalg.norm(des_pose[0:3]-reached_pose[0:3],ord=2)
                new_trial.position_err=pos_err
                new_trial.orient_err=q_err
                new_trial.angle_err=angle_err
                new_obj.traj[traj_n].trials.append(new_trial)
            else:
                empty+=1
        # Compute traj length:


    O_list=compute_traj_length()
    num=0
    for i in range(len(obj_list)):
        for j in range(len(O_list.arr)):
            if(obj_list[i].name==O_list.arr[j].name):
                t_no=O_list.arr[j].row%5
                obj_list[i].traj[t_no].length_position=float(O_list.arr[j].sol[0])              
                for k in range(len(obj_list[i].traj[t_no].trials)):
                    obj_list[i].traj[t_no].trials[k].norm_position_err=100.0*(obj_list[i].traj[t_no].trials[k].position_err/obj_list[i].traj[t_no].length_position)
                    obj_list[i].traj[t_no].trials[k].norm_orientation_err=100.0*(obj_list[i].traj[t_no].trials[k].orient_err/np.sqrt(2))
    
    b_list=copy.deepcopy(obj_list)
    #Pick 5 best errors:
    n=0
    for i in range(len(obj_list)):
        for t_no in range(len(obj_list[i].traj)):
            if(len(obj_list[i].traj[t_no].trials)>0):
                b_list[i].traj[t_no].trials=[]
                pos_arr=np.zeros(len(obj_list[i].traj[t_no].trials))
                junk_trials=[]
                for k in range(len(obj_list[i].traj[t_no].trials)):
                    # removing garbage values
                    pos_arr[k]=obj_list[i].traj[t_no].trials[k].norm_position_err
                    if(obj_list[i].traj[t_no].trials[k].norm_position_err>100.0):
                        junk_trials.append(k)
                        
                idx=pos_arr.argsort()[:5]
                for m in range(len(idx)):        
                    if len(junk_trials)>1:
                        if idx[m] not in junk_trials:
                            b_list[i].traj[t_no].trials.append(obj_list[i].traj[t_no].trials[idx[m]])
                    elif len(junk_trials)==1:
                        if idx[m]!=junk_trials[0]:
                            b_list[i].traj[t_no].trials.append(obj_list[i].traj[t_no].trials[idx[m]])
                    else:
                        b_list[i].traj[t_no].trials.append(obj_list[i].traj[t_no].trials[idx[m]])

    #Write to file
    f_name="new_best_5.csv"

    writer=csv.writer
    num=0



    with open(f_name,'w') as f:
        writer=csv.writer(f)
        header=['Object','Trajectory','Position Error','Normalized Position Error','Quaternion Error','Normalized Quaternion Error','Angle-axis Error']
        writer.writerow(header)
        for i in range(len(b_list)):
            for j in range(len(b_list[i].traj)):
                for k in range(len(b_list[i].traj[j].trials)):
                    data=list([b_list[i].name,j+1,b_list[i].traj[j].trials[k].position_err,b_list[i].traj[j].trials[k].norm_position_err,b_list[i].traj[j].trials[k].orient_err,b_list[i].traj[j].trials[k].norm_orientation_err,b_list[i].traj[j].trials[k].angle_err])
                    writer.writerow(data)
                    num+=1

#    print num
#    print empty

if __name__=='__main__':
    file_import('rerun_traj_err_min5.csv')
