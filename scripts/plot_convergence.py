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
#sns.color_palette(safeui)
#sns.palplot(sns.color_palette(safeui))



def compute_errors(obj):
    # Compute errors:
    for i in range(len(obj.arr)):
        des_pose=obj.arr[i].des
        reached_pose=obj.arr[i].sol
        
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
        obj.arr[i].err=err
    return obj

def plot_convergence(obj):
    #Load data into pandas
    p_err=[]
    r_err=[]
    names=[]
    weight=[]

    for i in range(len(obj.arr)):
        p_err.append(obj.arr[i].err[0])
        r_err.append(obj.arr[i].err[1])
        names.append(obj.arr[i].name)
        weight.append(obj.arr[i].weights[-1])


    # Adding an additional label for all objects   
    for i in range(len(obj.arr)):
        names.append('All objects')
        weight.append(obj.arr[i].weights[-1])
        p_err.append(obj.arr[i].err[0])
        r_err.append(obj.arr[i].err[1])

    data = pd.DataFrame()
    data['position']=p_err
    data['orientation']=r_err
    data['object']=names
    data['weight']=weight

    # Plot using the amazing seaborn:
    plt.figure(figsize=(27,10))
       
    
    sns.set(font_scale=1.45)
    sns.set_style("whitegrid")

    ax=sns.boxplot(y="orientation",x="object",hue='weight',data=data,orient="v",linewidth=1,palette=twoui,whis=10.0)
    ax=sns.swarmplot(y='orientation',x='object',data=data,hue='weight',orient="v",linewidth=1,palette=safeuiall,split=True)

    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.07),ncol=5, fancybox=False, shadow=False)


    plt.show()
 
        

if __name__=='__main__':
    file_name="../data/trajectories/waypoint_solution.csv"
    obj=TrajList(file_name,folder='')
    obj=compute_errors(obj)
    plot_convergence(obj)
    
    
