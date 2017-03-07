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
    method=sys.argv[1]
    f_name='../data/trajectories/general_'+method+'_solution.csv'
    trial_name=method+'_general.csv'
    folder='../data/trials/'
        
    objects=TrialList(trial_name,folder,n_trials)
    return objects

objects=read_objects()



f=h5py.File("rss_data.hdf5","w")
#odset=f.create_dataset("in_grasp_dataset",(),dtype='i')
n_trials=5
n_traj=5
for i in range(len(objects.arr)):
    obj=object.arr[i]
    obj_name=obj.name
    j=0
    for j in range(n_traj):
        data=np.array([
        j+=1
        i+=1
# Create a group for each object:
apple=f.create_group("apple")
# Create a dataset for each grasp:

grasp=f.create_dataset("grasp",(2,),grasp_type)

grasp["rigid_transform"]=np.zeros(6)
#grasp.create_group("desired_poses")

# Define data for single object:


#print f.name

#objects.create_dataset("
