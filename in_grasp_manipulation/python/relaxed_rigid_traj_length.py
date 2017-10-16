  #!/usr/bin/env python
from kdl_allegro_model import *
import time
from multiprocessing import Pool
import sys
from object_list import *
delta_t=1.0/60
T=10
method=sys.argv[1]
traj_folder='../data/trajectories/new_goal/'+method+'/'
  
def main():
    '''
    file="ral_same_poses.csv"
    folder_to_write="ral_lin/"
    obj_list=ObjectReachList(file,folder_to_write)  
    '''
    sc_pool=Pool()
    sta_t = time.time()
    tr=0

    trial_name=method+'.csv'
    folder='../data/trials/'
    
    obj_list=TrialList(trial_name,folder,5)

    for res in sc_pool.imap_unordered(multi_obj,obj_list.arr,chunksize=1):
        
        row=res
        #print row
        q=row[-1]-1
        obj_list.arr[q].length=row[0]
        # print obj_list.arr[q].sol
        tr+=1

    
    obj_list.write_traj_length(folder+method+'_traj_length.csv')

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

        joints_des=obj.des
        #Read joint trajectory from file:
        

        f_name=traj_folder+obj.f_name
        #folder=f_name
        f=open(f_name,'r')
        joint_traj=np.loadtxt(f)

        # Computing length of trajectory:
        position_length=0.0
        orient_length=0.0
        prev_pos=x0[0:3]
        err=0.0
        for i in range(1,len(joint_traj)):
            obj_pose=robot_model.object_pose(joint_traj[i],3)[0:3]
            err+=np.linalg.norm(obj_pose-prev_pos)
            prev_pos=obj_pose
        #position_length=np.linalg.norm(x0[0:3]-robot_model.object_pose(joint_traj[-1],3)[0:3])
        position_length=err
        return [position_length],i_number

    
if __name__=='__main__':
    main()
