'''
Builds the ILQR gain matrices
'''
from joint_traj_cost_obj_pose_states import *
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
    return obj

def compute_gains_in_grasp(obj):
    # The state x=[joint position, object position]
    # input u=[joint position]
    delta_t=1.0/60.0
    #print delta_t
    T=100
    robot_model=allegroKinematics(delta_t,T,fingers=obj.fing,fing=3)

    # object frame:
    D=obj.offset
    obj_frame=np.eye(4)
    obj_frame[0:3,3]=np.ravel(D)
    robot_model.joints=obj.grasp
    robot_model.object_frames=robot_model.update_object_frame(obj_frame,robot_model.joints,finger=3)

    robot_model.sim=inGraspSimVel(robot_model,0.1)
    #Weights:
    robot_model.w_x=2.0
    robot_model.w_u=0.0001
    robot_model.w_theta=1.0
    robot_model.n=robot_model.m+6
    ilqr=ILQR(robot_model,cost,final_cost)
    method='relaxed-rigidity'
    traj_folder='../data/trajectories/reachable_poses/'+method+'/'

    # Initial joint trajectory
    pre_plan=obj.open_loop_joint_traj#np.loadtxt(open(traj_folder+obj.f_name,'r'))
    # pre_plan has only joint trajectory, obtain object trajectory using the initial grasp:
    des_traj=[np.zeros(robot_model.n) for i in range(len(pre_plan))]
    for i in range(len(pre_plan)):
        des_traj[i][0:robot_model.m]=np.ravel(pre_plan[i])
        # getting object pose:
        des_traj[i][robot_model.m:]=robot_model.object_pose(pre_plan[i])
    x0=des_traj[0]

    x_des=des_traj
    
    u=np.zeros((T,robot_model.m))
    k_ff,K,x_new_arr,u_new=ilqr.controller(x0,x_des,u)
    obj.ilqr_joint_cost_K=K
    obj.ilqr_joint_cost_k_ff=k_ff
    obj.ilqr_joint_cost_x=x_new_arr
    obj.ilqr_joint_cost_u=u_new
    #obj.open_loop_joint_traj=pre_plan
    print obj.row
    return obj
        

def main():
    objects=pickle.load(open("../data/relaxed_rigidity.p",'rb'))
    sc_pool=Pool()
    sta_t = time.time()
    tr=0
    #compute_gains_in_grasp(objects[0])
    objects=objects[0:2]
    for res in sc_pool.imap_unordered(compute_gains_in_grasp,objects,chunksize=1):
        objects[res.row]=res
    elap=time.time()-sta_t
    print elap
    pickle.dump(objects,open('../data/relaxed_rigidity_ilqr_joint.p',"wb"))
   

    
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
