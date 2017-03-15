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
    in_grasp_sim=inGraspSimVel(robot_model,0.6,obj_noise=False,joint_noise=False)

    
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
    index=range(12,22,1)
    curr_x.append(x_new[index])
    obj_des.append(x_des[0][index])

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
        
        x_new=in_grasp_sim.get_next_state(x_new,u_new,err=True)
        

    # To correct for final state error, we use the ilqr gains at final state in a feedback controller for 100 timesteps, think of it as an integral controller to reduce steady state error
    
    for i in range(200):
        #prev_x=x_new
        t=-1
        u_dot=update_input(x_new,u[t],x[t],k_ff[t],K[t])
        u_new=u_dot
        curr_x.append(x_new[index])
        obj_des.append(x_des[t][index])
        x_new=in_grasp_sim.get_next_state(x_new,u_new)


    # TODO: Find a good way to plot multidimensional tracking.
    # Plot error:
    curr_x=np.matrix(curr_x)
    obj_des=np.matrix(obj_des)
    print curr_x.shape
    plot_pos=[1,2,3,4,5,6,7,9,10,11]
    row_pos=[0,0,0,0,1,1,1,2,2,2]
    col_pos=[0,1,2,3,0,1,2,0,1,2]
    plot_label=['joint-0','joint-1','joint-2','joint-3',
                'obj-x','obj-y','obj-z','obj-theta_x','obj-theta_y','obj-theta_z']
    j=0
    fig, axes = plt.subplots(3, 4,sharex=True,figsize=(20,10))
    for i in index:
        ax=axes[row_pos[j]][col_pos[j]]
        #ax=plt.subplot(3,4,plot_pos[j])
        ax.plot(range(len(obj_des)),obj_des[:,i-12],label='desired value')
        ax.plot(range(len(curr_x)),curr_x[:,i-12],label='value from simulation')
        ax.set_xlabel('timestep')
        ax.set_ylabel(plot_label[j])
        j+=1
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center', bbox_to_anchor=(0.5, 1.0),ncol=2, fancybox=False, shadow=False)
    
    pad=5
    rows = ['Thumb \nJoints','Object \n Position','Object\nOrientation']
    for ax, row in zip(axes[:,0], rows):
        ax.annotate(row, xy=(0, 0.5), xytext=(-ax.yaxis.labelpad - pad, 0),
                    xycoords=ax.yaxis.label, textcoords='offset points',
                    size='large', ha='right', va='center')

    
    plt.tight_layout()
    fig.subplots_adjust(left=0.1, top=0.95)
    plt.show()
    ind=1
    fin_idx=99
    vmin=np.min([np.min(K[ind]),np.min(K[fin_idx])])
    vmax=np.max([np.max(K[ind]),np.max(K[fin_idx])])
    #fig=plt.figure()
    fig,ax=plt.subplots(2)#add_subplot(111)
    cax=ax[0].imshow(K[ind],cmap='plasma',origin='lower',vmin=vmin, vmax=vmax)
    ax[0].set_xlabel('States')
    ax[0].set_ylabel('Joint input')
    ax[0].xaxis.set_ticks(np.arange(0,22, 2.0))
    ax[0].set_title('K matrix at time= '+str(ind))
            
    cax_1=ax[1].imshow(K[fin_idx],cmap='plasma',origin='lower',vmin=vmin, vmax=vmax)
    ax[1].set_xlabel('States')
    ax[1].set_ylabel('Joint input')
    ax[1].xaxis.set_ticks(np.arange(0,22, 2.0))
    ax[1].set_title('K matrix at time= '+str(fin_idx+1))

    '''
    cax=ax[2].imshow(k_ff.T,cmap='plasma',origin='lower')
    ax[2].set_xlabel('States')
    ax[2].set_ylabel('Joint input')
    #ax[2].xaxis.set_ticks(np.arange(0,22, 2.0))
    '''
    fig.subplots_adjust(right=0.7)
    cbar_ax=fig.add_axes([0.85, 0.15, 0.05, 0.7])
    fig.colorbar(cax, cax=cbar_ax)
    #fig.tight_layout()
    plt.show()
    
def main():
    # read data from pickle file:
    objects=pickle.load(open("../data/relaxed_rigidity_ilqr_joint.p",'rb'))
    # simulate robot and plot error:
    sim_robot(objects[1])

main()
