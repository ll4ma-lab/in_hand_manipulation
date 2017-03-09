'''
Builds the ILQR gain matrices
'''
from joint_traj_cost import *
from ilqr_traj_parallel import *
from kdl_allegro_model import *
from dynamics_sim import *
import pickle
from multiprocessing import Pool
import time
def deinterpolate_pre_plan(interpolated_plan,T):
    pre_plan=np.zeros((10,16))
    for i in range(len(interpolated_plan)):
        if(i%10==0):
                pre_plan[i/10]=interpolated_plan[i]
    pre_plan=pre_plan[0:T]
    return pre_plan


def compute_gains(obj):
    delta_t=1.0/60.0
    #print delta_t
    T=100
    robot_model=allegroKinematics(delta_t,T,fingers=obj.fing)
    robot_model.sim=SimAllegroJoint(delta_t)
    #Weights:
    robot_model.w_x=2.0
    robot_model.w_u=0.0001
    robot_model.n=robot_model.m
    ilqr=ILQR(robot_model,cost,cost_gradient,final_cost,final_cost_gradient)
    method='relaxed-rigidity'
    traj_folder='../data/trajectories/reachable_poses/'+method+'/'

    # Initial joint trajectory
    pre_plan=obj.open_loop_joint_traj#np.loadtxt(open(traj_folder+obj.f_name,'r'))
    '''
    #pre_plan=deinterpolate_pre_plan(pre_plan,T)
    x0=pre_plan[0]
    #print x0
    x_des=pre_plan#[pre_plan[0] for k in range(T)]
    u=np.zeros((T,robot_model.m))
    k_ff,K,x_new_arr,u_new=ilqr.controller(x0,x_des,u)
    obj.ilqr_joint_cost_K=K
    obj.ilqr_joint_cost_k_ff=k_ff
    obj.ilqr_joint_cost_x=x_new_arr
    obj.ilqr_joint_cost_u=u_new
    #obj.open_loop_joint_traj=pre_plan
    '''
    return obj
        

def main():
    objects=pickle.load(open("../data/relaxed_rigidity.p",'rb'))
    sc_pool=Pool()
    sta_t = time.time()
    tr=0
    for res in sc_pool.imap_unordered(compute_gains,objects,chunksize=len(objects)/10):
        objects[res.row]=res
    elap=time.time()-sta_t
    print elap
    pickle.dump(objects,open('../data/relaxed_rigidity_ilqr.p',"wb"))
        
#objects[0]=compute_gains(objects[0])

    
def debug():
    delta_t=1.0/60.0
    #print delta_t
    T=100
    robot_model=allegroKinematics(delta_t,T,fingers=[0,1,2,3])
    robot_model.sim=SimAllegroJoint(delta_t)
    #Weights:
    robot_model.w_x=2.0
    robot_model.w_u=0.0001
    robot_model.n=robot_model.m
    ilqr=ILQR(robot_model,cost,cost_gradient,final_cost,final_cost_gradient)
    print 'ILQR is setup'
    pre_plan=np.load('new_plan.npy')
    #pre_plan=deinterpolate_pre_plan(pre_plan,T)
    x0=pre_plan[0]
    print x0
    x_des=pre_plan#[pre_plan[0] for k in range(T)]
    u=np.zeros((T,robot_model.m))
    k_ff,K,x_new_arr,u_new=ilqr.controller(x0,x_des,u)
    # Write ilqr to files:
    np.save("k.npy",K)
    np.save("k_ff.npy",k_ff)
    np.save("x.npy",x_new_arr)
    np.save("u.npy",u_new)
    #ilqr.controller(x0,x_des,u_n)

main()
