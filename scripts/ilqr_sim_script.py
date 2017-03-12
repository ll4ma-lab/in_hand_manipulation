# Runs the trajectory on the robot with feedback using ILQR gains.
import numpy as np
#from robot_node import *
import matplotlib as mpl
import matplotlib.pyplot as plt
import pickle
from kdl_allegro_model import *
from dynamics_sim import *


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
    
    # object frame:
    D=obj.offset
    obj_frame=np.eye(4)
    obj_frame[0:3,3]=np.ravel(D)
    
    robot_model.joints=obj.grasp
    robot_model.object_frames=robot_model.update_object_frame(obj_frame,robot_model.joints,finger=3)
    in_grasp_sim=inGraspSimVel(robot_model,100.0)

    
    # Load ilqr gains:
    x=obj.ilqr_joint_cost_x
    u=obj.ilqr_joint_cost_u
    k_ff=obj.ilqr_joint_cost_k_ff
    K=obj.ilqr_joint_cost_K

    dt=1.0/60.0    
    x_new=in_grasp_sim.publish_grasp(robot_model.joints)#allegro.joint_states.position
    t=0
    #print x_new
    #exit()
    u_dot=update_input(x_new,u[t],x[t],k_ff[t],K[t])
    # Compute new position based on velocity and loop rate
        
    u_new=x[0][:16]#u_dot*dt#x_new[:16]+u_dot*dt
    x_new=in_grasp_sim.publish_grasp(u_new)
    curr_x=[]
    #curr_x.append(0.0)
    #des_x=x
    #time_=range(100)

    pre_plan=obj.open_loop_joint_traj
    #print pre_plan[0]
    robot_model.n=robot_model.m+6
    des_traj=[np.zeros(robot_model.n) for i in range(len(pre_plan))]
    for i in range(len(pre_plan)):
        des_traj[i][0:robot_model.m]=np.ravel(pre_plan[i])
        # getting object pose:
        des_traj[i][robot_model.m:]=robot_model.object_pose(pre_plan[i])

    x_des=des_traj#obj.open_loop_joint_traj
    #x_des=des_traj[:][17]
    obj_des=[]
    index=19
    for i in range(1,99):
        t=i
        # Get updated input  from ilqr controller:
        u_dot=update_input(x_new,u[t],x[t],k_ff[t],K[t])

        # For debugging robot simulator, uncomment below line:
        #u_dot=(x_des[t][:16]-x_new[:16])/dt

        #ILQR outputs a joint velocity with delta_t from robot model used in computing ILQR
        u_new=u_dot

        # Storing current state and desired state for plotting:
        # TODO: store in pickle database
        curr_x.append(x_new[index])
        obj_des.append(x_des[t][index])

        # Storing norm error for debugging:
        #curr_x.append(np.linalg.norm(x[t][:16]-x_new[:16]))

        # Sending new joint command to simulation:
        # Can be replaced with actual robot, with robot in joint velocity mode:
        x_new=in_grasp_sim.get_next_state(x_new,u_new)
        

    # To correct for final state error, we use the ilqr gains at final state in a feedback controller for 100 timesteps, think of it as an integral controller to reduce steady state error
    
    for i in range(100):
        prev_x=x_new
        t=-1
        u_dot=update_input(x_new,u[t],x[t],k_ff[t],K[t])
        u_new=u_dot
        curr_x.append(x_new[index])
        obj_des.append(x_des[t][index])
        x_new=in_grasp_sim.get_next_state(x_new,u_new)


    # TODO: Find a good way to plot multidimensional tracking.
    # Plot error:
    #x_err=abs(des_x-curr_x)
    #curr_x=np.ravel(curr_x)
    #print curr_x[0]
    plt.plot(range(len(obj_des)),obj_des,label='desired value')
    plt.plot(range(len(curr_x)),curr_x,label='value from simulation')
    plt.legend()
    
    #plt.plot(time_, curr_x)
    plt.show()

def main():
    # read data from pickle file:
    objects=pickle.load(open("../data/relaxed_rigidity_ilqr_joint.p",'rb'))
    # simulate robot and plot error:
    sim_robot(objects[0])

def compare_error():
    raw=np.load('raw.npy')
    ilqr_err=np.load('ilqr.npy')
    time_=range(100)
    # sum errors in all joints:
    sum_err=np.linalg.norm(raw,axis=1)
    sum_ilqr_err=np.linalg.norm(ilqr_err,axis=1)
    ax=plt.plot(range(len(sum_ilqr_err)),sum_err,label='Raw Control')
    ay=plt.plot(range(len(sum_ilqr_err)),sum_ilqr_err,label='ILQR feedback')
    plt.title('Trajectory tracking')
    plt.xlabel('timestep')
    plt.ylabel('Error norm across all joints')
    plt.legend()
    plt.show()

def plot_error():
    
    ilqr_err=np.load('ilqr.npy')
    #time_=range(100)
    # sum errors in all joints:
    j=0
    for i in range(len(ilqr_err[0,:])):
        if(ilqr_err[-2,i])>0.01:
            j=i
            print j
    
        
    ay=plt.plot(range(len(ilqr_err)),ilqr_err[:,j],label='ILQR feedback')
    plt.title('Trajectory tracking')
    plt.xlabel('timestep')
    plt.ylabel('Joint error(rad)')
    #plt.legend()
    plt.show()

#compare_error()
#plot_error()

main()
