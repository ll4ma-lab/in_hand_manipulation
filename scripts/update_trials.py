from object_list import *


if __name__=='__main__':
    method='relaxed-position'
    trial_name='../data/trials/'+method+'.csv'
    solution_name='../data/trajectories/fork_relaxed-position_solution.csv'
    obj=TrajList(solution_name,'')
    trial=TrialList(trial_name,'',5)
    #new_traj_names=['Fork','Cube-2f','Lego-2f','Spatula','Jello-2f']

    for i in range(len(obj.arr)):
        for j in range(len(trial.arr)):
                if trial.arr[j].f_name==obj.arr[i].f_name:
                    trial.arr[j].sol=obj.arr[i].sol
                    trial.arr[j].des=obj.arr[i].des
                    trial.arr[j].grasp=obj.arr[i].grasp
                    #print k
                    print obj.arr[i].f_name
        

    trial.write_reached_pose('../data/trials/relaxed-position.csv')

