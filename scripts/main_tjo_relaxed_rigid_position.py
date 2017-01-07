#!/usr/bin/env python
from cost_relaxed_rigid_obj_sampled import *
from kdl_allegro_model import *
import traj_opt as tjO
from interpolate_traj import *
import time
from multiprocessing import Pool

from object_list import *
delta_t=1.0/60
T=10
def main():
    method='relaxed-position'
    file="../data/goal_poses/feas_goal_poses.csv"
    folder_to_write="../data/trajectories/exp_2/"+method+"/"
    obj_list=ObjectList(file,folder_to_write)  
    obj_list.get_weights('../data/weights/exp_2_relaxed_position.csv')
    print len(obj_list.arr)
    sc_pool=Pool()
    sta_t = time.time()
    tr=0
    for res in sc_pool.imap_unordered(multi_obj,obj_list.arr,chunksize=1):
        row=res
        q=row[-1]
        obj_list.arr[q].sol=row[0]
        tr+=1
    
    elap=time.time()-sta_t
    print "Time to compute(seconds): "+str(elap)
    obj_list.write_sol_from_tjo("../data/trajectories/exp_2_"+method+"_solution.csv")

def multi_obj(obj):

        i_number=obj.row
        # Input to the optimization by the user
        D=obj.offset # Distance of the object from finger  

        # initial joint angles    
        joints=obj.grasp
        obj_name=obj.name

        robot_model=allegroKinematics(delta_t,T,fingers=obj.fing)
        robot_model.joints=joints
        robot_model.object_frames=robot_model.add_object_frame(D,joints,finger=3)    

        robot_model.final_obj_dim_weights=np.array([100.0,100.0,100.0,-1.0,-1.0,1.0])

        robot_model.obj_dim_weights=np.array([100.0,100.0,100.0,-1.0,-1.0,1.0])

        robot_model.psi=[0.0,1.0,0.0]
        #print obj.weights
        robot_model.wp=obj.weights[0]
        robot_model.wr=obj.weights[1]/float(T)
        robot_model.wo=0.0#obj.weights[2]/float(T)
        if(obj. weights[3]!=0.0):
            robot_model.wl=obj.weights[3]/float(T)
        else:
            robot_model.wl=0.0
        u0=joints.copy()

        max_iter=int(obj.weights[4])
        # Initial pose of the object:
        x0=robot_model.object_pose(joints,3)
        #print x0

        x_des=obj.des
            
        # Storing desired object pose for faster solution:
        obj_mat=np.eye(4)
        R=PyKDL.Rotation.RPY(x_des[3],x_des[4],x_des[5])
        for k in range(3):
            for i in range(3):
                obj_mat[k,i]=R[k,i]
        obj_mat[0:3,3]=x_des[0:3]
        robot_model.des_finger_pose=robot_model.object_finger_diff(obj_mat,3)   

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
            linear_poses.append(list(robot_model.object_finger_diff(x_des_mat,3)))

        

        robot_model.linear_finger_poses=linear_poses
        u_input=np.array([joints.copy() for k in range(T+1)]) 

        x_d=np.array(list(np.array([x_des for k in range(T)])))
        
        
        opt_options={'Iterations limit':max_iter,'Solution':'No','iPrint':0,'iSumm':0}

        opta=tjO.TrajOpt(u0,x_d,u_input,robot_model,cost,final_cost,cost_gradient,
                         final_cost_gradient,"SNOPT",opt_options)

        u_new,_=opta.optimize()  
    
        x_final=opta.robot.object_pose(u_new[-1])
    


        interpolate_joints(u_new,obj.folder+obj.f_name)
        list_arr=[x_final,i_number]

        print 'Desired '+str(i_number)
        print x_des
        print x_final
        
        return x_final,i_number    

    
if __name__=='__main__':
    main()
