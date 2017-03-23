# Runs the trajectory on the robot with feedback using ILQR gains.
import numpy as np
#from robot_node import *
import matplotlib as mpl
import matplotlib.pyplot as plt
import pickle
from kdl_allegro_model import *
from dynamics_sim import *
from interpolate_traj import *

mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42
#mpl.rcParams.update({'font.size': 42})

#mport seaborn as sns
#afeuiall = ['#ffffff','#fef0d9','#fdcc8a','#fc8d59','#d7301f']
#import rospy


def update_input(curr_state,u,x,k_ff,K):
    #print np.dot(K,curr_state-x)+k_ff
    #print curr_state
    u_dot=u-k_ff-np.dot(K,curr_state-x)
    return u_dot

def sim_robot(obj):
    #allegro=robotStates(listen_aruco=False)
    # robot_model:
    delta_t=1.0/60.0
    dt=delta_t
    #print delta_t
    T=100
    robot_model=allegroKinematics(delta_t,T,fingers=obj.fing,fing=3)
    '''
    # object frame:
    D=obj.offset
    obj_frame=np.eye(4)
    obj_frame[0:3,3]=np.ravel(D)
    
    #robot_model.joints=obj.grasp
    #robot_model.object_frames=robot_model.update_object_frame(obj_frame,robot_model.joints,finger=3)
    #in_grasp_sim=inGraspSimVel(robot_model,0.6,obj_noise=True,joint_noise=False)
    '''
    finger=0
    finger_pose=[]
    finger_pose_interp=[]

    plan=obj.open_loop_joint_traj
    deinterp_plan=deinterpolate_pre_plan(plan,10,obj.grasp)
    # get new plan
    plan=interpolate_joints(deinterp_plan,'n',False)
    for i in range(100):
        t=i
        u=plan[i]
        new_pose=robot_model.end_effector_pose_array(u[finger*4:finger*4+4],finger)
        finger_pose_interp.append(new_pose[3])
    #print len(deinterp_plan)
    
    for i in range(len(deinterp_plan)):
        t=i
        u=deinterp_plan[i]
        new_pose=robot_model.end_effector_pose_array(u[finger*4:finger*4+4],finger)
        finger_pose.append(new_pose[3])
    
    print len(finger_pose)
    time_=np.array(range(len(finger_pose)))*10-1
    time_[0]=0
    plt.plot(time_,finger_pose,marker='v',label='Original Trajectory')
    plt.plot(np.array(range(len(finger_pose_interp))),finger_pose_interp,label='Interpolated Trajectory')
    plt.xlabel('Timestep')
    plt.ylabel('Fingertip Theta_X')
    plt.legend()
    plt.show()
def main():
    plt.style.use('seaborn-poster')

    # read data from pickle file:
    objects=pickle.load(open("../data/relaxed_rigidity_ilqr_joint.p",'rb'))
    # simulate robot and plot error:
    sim_robot(objects[0])

main()
