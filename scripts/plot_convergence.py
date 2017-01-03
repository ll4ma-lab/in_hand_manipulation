# Computes orientation error between frames
import numpy as np
import csv
import PyKDL
from object_list import TrajList
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42


import seaborn as sns
#plt.style.use('seaborn-white')
#safeui = ['#fef0d9','#fdcc8a','#fc8d59','#d7301f']
safeuiall=['#e41a1c','#377eb8','#4daf4a','#984ea3','#ff7f00']
twoui=['#a6611a','#dfc27d','#f5f5f5','#80cdc1','#018571']
safeui=['#fdcc8a','#377eb8']
singleui=['#696969','#696969','#696969','#696969']
#sns.color_palette(safeui)
#sns.palplot(sns.color_palette(safeui))



def compute_errors(obj):
    names=[]
    # Compute errors:
    for i in range(len(obj)):
        des_pose=obj[i].des
        reached_pose=obj[i].sol
        
        R_des=PyKDL.Rotation.RPY(des_pose[3],des_pose[4],des_pose[5])
        R_reach=PyKDL.Rotation.RPY(reached_pose[3],reached_pose[4],reached_pose[5])

            
        des=np.array(R_des.GetQuaternion())
        reach=np.array(R_reach.GetQuaternion())
        q_sum=np.linalg.norm(des+reach,ord=2)
        q_diff=np.linalg.norm(des-reach,ord=2)
            
        R_err=np.min([q_sum,q_diff])
        #position error:
        pos_err=np.linalg.norm(des_pose[0:3]-reached_pose[0:3],ord=2)
        err=[pos_err,R_err]
        
        if(pos_err>0.010):
            names.append(obj[i].f_name)
            #err=[0.0,0.0]
        obj[i].err=err    
    return obj,names

def plot_convergence(obj):
    #Load data into pandas
    p_err=[]
    r_err=[]
    names=[]
    weight=[]

    for i in range(len(obj)):
        p_err.append(obj[i].err[0])
        r_err.append(obj[i].err[1])
        names.append(obj[i].name)
        if(i<50):
            weight.append('bias-cost(5k)')
        elif(50<=i<50*2):
            weight.append('no bias-cost(20k)')
        elif(50*2<=i<50*3):
            weight.append('no bias-cost(20k)')
        elif(50*3<=i<50*4):
            weight.append('bias-cost(20k)')

    # Adding an additional label for all objects   
    for i in range(len(obj)):
        names.append('All objects')
        if(i<50):
            weight.append('bias-cost(5k)')
        elif(50<=i<50*2):
            weight.append('no bias-cost(20k)')
        elif(50*2<=i<50*3):
            weight.append('no bias-cost(20k)')
        elif(50*3<=i<50*4):
            weight.append('bias-cost(20k)')
        p_err.append(obj[i].err[0])
        r_err.append(obj[i].err[1])

    data = pd.DataFrame()
    data['position']=p_err
    data['orientation']=r_err
    data['object']=names
    #data['method']=method
    data['weight']=weight

    # Plot using the amazing seaborn:
    plt.figure(figsize=(8,14))
       
    
    sns.set(font_scale=1.45)
    sns.set_style("whitegrid")

    ax=sns.boxplot(x="position",y="object",data=data,orient="h",hue='weight',linewidth=1,palette=safeuiall,whis=10.0)
    ax=sns.swarmplot(x='position',y='object',data=data,orient="h",hue='weight',linewidth=1,palette=singleui,split=True,edgecolor='black')
    ax.set(ylabel='Objects', xlabel='Position Error')
    handles, labels = ax.get_legend_handles_labels()
    
    ax.legend(handles[0:2], labels[0:2],title='bias cost(max. iterations)',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=2, fancybox=False, shadow=False)
    plt.xlim(0)
    #plt.savefig('/home/bala/Dropbox/convergence_exp_1_position_error.png', bbox_inches='tight')
    plt.show()
 
def plot_convergence_method(obj,n=0):
    #Load data into pandas
    p_err=[]
    r_err=[]
    names=[]
    method=[]
    
    for i in range(len(obj)):
        p_err.append(obj[i].err[0])
        r_err.append(obj[i].err[1])
        names.append(obj[i].name)
        if(i<n):
            method.append('IK-rigid')
        elif(n<=i and i<2*n):
            method.append('Trinkle')
        elif(2*n<=i and  i<3*n):
            method.append('Relaxed-position')
        elif(3*n<=i and i<4*n):
            method.append('Relaxed-full')

    '''
    # Adding an additional label for all objects   
    for i in range(len(obj)):
        names.append('All objects')
        if(i<n):
            method.append('IK-rigid')
        elif(n<=i and i<2*n):
            method.append('Trinkle')
        elif(2*n<=i and i<3*n):
            method.append('Relaxed-position')
        elif(3*n<=i and i<4*n):
            method.append('Relaxed-full')


        p_err.append(obj[i].err[0])
        r_err.append(obj[i].err[1])
    '''
    data = pd.DataFrame()
    data['position']=p_err
    data['orientation']=r_err
    data['object']=names
    data['method']=method

    # Plot using seaborn:
    plt.figure(figsize=(8,15))
       
    
    sns.set(font_scale=1.45)
    sns.set_style("whitegrid")

    ax=sns.boxplot(x="position",y="object",data=data,hue="method",orient="h",linewidth=1,palette=safeuiall,whis=10.0)
    #ax=sns.swarmplot(x='position',y='object',data=data,orient="h",hue="method",linewidth=1,palette=singleui,split=True)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],title='Method',loc='upper center', bbox_to_anchor=(0.5, 1.07),ncol=5, fancybox=False, shadow=False)
    ax.set(xlabel='Position Error', ylabel='Objects')
        
    plt.savefig('convergence_exp_2_position_error.eps', bbox_inches='tight')
    #plt.show()

        

if __name__=='__main__':


    
    obj_3=TrajList("../data/trajectories/exp_2_relaxed-full_solution.csv",folder='')
    traj_n=len(obj_3.arr)
    '''
    obj_2=TrajList('../data/trajectories/exp_2_relaxed-position_solution.csv','')
    obj_0=TrajList('../data/trajectories/exp_2_ik-rigid_solution.csv','')
    obj_1=TrajList('../data/trajectories/exp_2_trinkle_solution.csv','')
    obj_3=TrajList("../data/trajectories/exp_2_relaxed-full_solution.csv",folder='')
    traj_n=len(obj_3.arr)
    obj_all=obj_0.arr+obj_1.arr+obj_3.arr+obj_3.arr
    '''
    #obj_all=TrajList("../data/trajectories/waypoint_solution.csv",'').arr
    
    obj,n=compute_errors(obj_3.arr)
    print n
    plot_convergence(obj)
    #print n
    #plot_convergence_method(obj,traj_n)

    
