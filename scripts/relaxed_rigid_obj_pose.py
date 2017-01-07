 #!/usr/bin/env python
from kdl_allegro_model import *
import time
from multiprocessing import Pool

from object_list import *
delta_t=1.0/60
T=10
def main():
    file="../data/goal_poses/feas_goal_joint_angles.csv"
    folder_to_write=""
    obj_list=ObjectReachList(file,folder_to_write)  

    for i in range(len(obj_list.arr)):
        grasp=obj_list.arr[i].grasp
        grasp[1]+=0.1
        grasp[5]+=0.1
        obj_list.arr[i].grasp=grasp

    sc_pool=Pool()
    sta_t = time.time()
    tr=0

    for res in sc_pool.imap_unordered(multi_obj,obj_list.arr,chunksize=10):
        
        row=res
        q=row[-1]
        obj_list.arr[q].sol=row[0]
        tr+=1

    obj_list.write_sol_from_tjo('../data/goal_poses/feas_goal_poses.csv')

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

        robot_model.obj_dim_weights=np.array([1.0,1.0,1.0,-1.0,-1.0,1.0])


        
        
        robot_model.wp=1000.0
        robot_model.wr=20.0
        robot_model.wo=0.01
        u0=joints.copy()


        # Initial pose of the object:
        x0=robot_model.object_pose(joints,3)
        #print x0
        joints_des=obj.des

        
        return robot_model.object_pose(joints_des,3),i_number

    
if __name__=='__main__':
    main()
