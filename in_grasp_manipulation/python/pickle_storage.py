import pickle

from object_list import *



def read_objects():
    method='relaxed-rigidity'
    f_name='../data/trajectories/reachable_poses/'+method+'.csv'
    trial_name=method+'.csv'
    folder='../data/trials/'
    n_trials=5
        
    objects=TrialList(trial_name,folder,n_trials)
    return objects

objects=read_objects().get_traj_list(5)


pickle.dump(objects,open('../data/relaxed_rigidity.p','wb'))
