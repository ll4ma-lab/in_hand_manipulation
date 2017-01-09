from object_list import *

if __name__=='__main__':
    trial_name='../data/trials/relaxed-full.csv'
    solution_name='../data/trajectories/exp_2_relaxed-full_solution.csv'
    obj=TrajList(solution_name,'')
    trial=TrialList(trial_name,'',5)
    new_traj_names=['Fork_4_44', 'Fork_6_46', 'Fork_9_49', 'Plane_3_63', 'Screwdriver_1_71', 'Screwdriver_4_74', 'Spatula_4_84', 'Cube-2f_1_121']

    for i in range(len(obj.arr)):
        for j in range(len(trial.arr)):
            for k in new_traj_names:                
                if trial.arr[j].f_name==k and obj.arr[i].f_name==k:
                    trial.arr[j].sol=obj.arr[i].sol
                    trial.arr[j].des=obj.arr[i].des

    trial.write_reached_pose('../data/trials/new.csv')

