# This file was used to store all the previously generated data into h5py.
import h5py
import numpy as np
from data_types import *
from object_list import *
import sys
'''
Terms in h5py:
groups: folders
datasets: numpy like arrays (can have only numbers)

each group as an object?
each dataset is a grasp
-each desired pose is a subgroup
   -each subgroup can have a subgroup for trajectory generation
   -each subgroup can have execution data
   -each subgroup can have feedback policy


/apple.graspinfo
/apple/desired_pose/pose/method(solution, trajectory, object pose)/execution(trialData-(reached pose,joint states,object states,bagfile_name)
'''


def read_objects():
    method='relaxed-rigidity'
    f_name='../data/trajectories/reachable_poses/'+method+'.csv'
    trial_name=method+'.csv'
    folder='../data/trials/'
    n_trials=5
        
    objects=TrialList(trial_name,folder,n_trials)
    return objects

objects=read_objects()



f=h5py.File("../data/rss_data.hdf5","w")
method='relaxed-rigidity'
traj_folder='../data/trajectories/reachable_poses/'+method+'/'
n_methods=len(method)
#odset=f.create_dataset("in_grasp_dataset",(),dtype='i')
n_traj=50 # number of trajectories per object
n_trials=5
prev_name='temp'
i=0
rig_fing=np.ravel([3,-1,-1,-1])
while i<len(objects.arr):
    obj=objects.arr[i]
    obj_name=obj.name
    if(prev_name!=obj_name):
        print obj_name
        print i
        prev_name=obj_name
        # create a new group
        new_object=f.create_group(obj_name)
        # create dataset for the new object 
        # Each dataset entry stores a desired pose and its trajectory.
        data=new_object.create_dataset("data",(n_traj,),grasp_type)

    # Read from traj file:
    traj=np.loadtxt(open(traj_folder+obj.f_name,'r'))
    
    # TODO: use kdl to get obj traj
    obj_traj=np.zeros((100,6))

    # This data needs to be stored per method of trajectory generation, currently it only stores one method.
    # TODO: add all other cost formulations to this dataset
    K=np.zeros((100,16,16))
    k_ff=np.zeros((100,6))
    x=np.zeros((100,16))
    u=np.zeros((100,16))
    
    ilqr_data=np.array([K ,k_ff , x ,u],dtype=ilqr_type)    
    j=0
    trials=[]
    for j in range(n_trials):
        # Add dataset data:
        trial_data=np.array([(np.ravel(objects.arr[i].reached))],dtype=trial_type)
        trials.append(trial_data)
        i+=1
    trials=np.array(trials,dtype=trial_type)    
    trajectory_data=np.array([method,obj.sol,traj,obj_traj,trials,ilqr_data],dtype=trajectory_type)
   
    contact_fingers=np.ravel(obj.fing+[-1]*(4-len(obj.fing)))
    rigid_offset=np.ravel(list(obj.offset)+list([0.0,0.0,0.0]))
    grasp_data=np.array([np.ravel(obj.grasp),contact_fingers,rig_fing,rigid_offset,obj.initial_pose,obj.des,trajectory_data],dtype=grasp_type)
    #data[i%5]=grasp_data
#Close h5py file
f.close()

