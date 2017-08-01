# Runs the trajectory on the robot with feedback using ILQR gains.
import numpy as np
#from robot_node import *
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42
#mpl.rcParams.update({'font.size': 42})

#mport seaborn as sns
#afeuiall = ['#ffffff','#fef0d9','#fdcc8a','#fc8d59','#d7301f']
#import rospy

def update_input(curr_state,u,x,k_ff,K):
    #print np.dot(K,curr_state-x)+k_ff
    u_dot=u-k_ff-np.dot(K,curr_state-x)
    return u_dot
def main():
    #allegro=robotStates(listen_aruco=False)
    allegro=robotSim()
    # Load ilqr gains:
    x=np.load("x.npy")
    u=np.load("u.npy")
    k_ff=np.load("k_ff.npy")
    K=np.load("k.npy")
    raw_input("go to initial grasp")
    pre_plan=np.load('new_plan.npy')
    #allegro.position_mode()
    allegro.publish_grasp(pre_plan[0])
    raw_input("run ilqr for initial grasp")
    dt=1.0/60.0    
    x_new=allegro.joint_states.position
    t=0
    u_dot=update_input(x_new,u[t],x[t],k_ff[t],K[t])
    # Compute new position based on velocity and loop rate
        
    u_new=x_new+u_dot*dt
    allegro.publish_joint(u_new)
    raw_input("run full traj")
    curr_x=[]
    curr_x.append(np.zeros(16))
    des_x=x
    #time_=range(100)
    x=pre_plan
    for i in range(99):
        x_new=allegro.joint_states.position
        prev_x=x_new
        t=i
        u_dot=update_input(x_new,u[t],x[t],k_ff[t],K[t])
        # Compute new position based on velocity and loop rate
        
        #u_new=x[t]#+u_dot*dt
        u_new=x[t]#+u_dot*dt
        curr_x.append(x[t]-x_new)

        allegro.publish_joint(u_new)

    for i in range(100):
        x_new=allegro.joint_states.position
        prev_x=x_new
        t=-1
        u_dot=update_input(x_new,u[t],x[t],k_ff[t],K[t])
        # Compute new position based on velocity and loop rate
        
        #u_new=x[t]#+u_dot*dt
        u_new=x[t]#+u_dot*dt
        curr_x.append(x[t]-x_new)
        #print u_new
        allegro.publish_joint(u_new)

    # Plot error:
    #x_err=abs(des_x-curr_x)
    np.save('r aw.npy',curr_x)
    curr_x=np.matrix(curr_x)
    plt.plot(range(len(curr_x)), curr_x)
    #plt.plot(time_, curr_x)
    plt.show()

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

compare_error()
#plot_error()

#main()
