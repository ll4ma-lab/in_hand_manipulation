# Computes orientation error between frames
import numpy as np
import csv
import PyKDL
from textwrap import wrap

from object_list import TrajList

from object_list import TrialList
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42
#mpl.rcParams.update({'font.size': 42})

import seaborn as sns
#plt.style.use('seaborn-white')
safeuiall=['#ffffff','#f0f9e8','#bae4bc','#7bccc4','#2b8cbe']
safeuiall_RSS = ['#ffffff','#fef0d9','#fdcc8a','#fc8d59','#d7301f']
safeui_1 = ['#fef0d9','#fdcc8a','#fc8d59','#d7301f']
#safeuiall=['#e41a1c','#377eb8','#4daf4a','#984ea3','#ff7f00']
twoui=['#a6611a','#dfc27d','#f5f5f5','#80cdc1','#018571']
safeui=['#fdcc8a','#377eb8']
singleui=['#696969','#696969','#696969','#696969']
#sns.color_palette(safeui)
#sns.palplot(sns.color_palette(safeui))
twoui=['#f1a340','#998ec3']
#fourui=['#fee6ce','#fdae6b','#5e3c99','#b2abd2']
fourui=['#e66101','#fdb863','#5e3c99','#b2abd2']
ten_colors=['#a50026','#d73027','#f46d43','#fdae61','#fee090','#e0f3f8','#abd9e9','#74add1','#4575b4','#313695']

def compute_initial_errors(obj):
    names=[]
    # Compute errors:
    for i in range(len(obj)):
        if(obj[i].length!=[]):
            des_pose=obj[i].des
            reached_pose=obj[i].initial_pose
            
            R_des=PyKDL.Rotation.RPY(des_pose[3],des_pose[4],des_pose[5])
            R_reach=PyKDL.Rotation.RPY(reached_pose[3],reached_pose[4],reached_pose[5])

            
            des=np.array(R_des.GetQuaternion())
            reach=np.array(R_reach.GetQuaternion())
            q_sum=np.linalg.norm(des+reach,ord=2)
            q_diff=np.linalg.norm(des-reach,ord=2)
            
            R_err=100*np.min([q_sum,q_diff])/np.sqrt(2)
            #position error:
            pos_err=100*np.linalg.norm(des_pose[0:3]-reached_pose[0:3],ord=2)#/obj[i].length
            err=[pos_err,R_err]        
            obj[i].init_err=err

        
    return obj,names

def compute_errors(obj):
    names=[]
    E_pos=[]
    # Compute errors:
    for i in range(len(obj)):
        print obj[i].sol
        if(obj[i].sol!=[] and not('2f' in obj[i].name)):
            des_pose=obj[i].des
            reached_pose=obj[i].sol
            #initial_pose=obj[i].initial_pose
            initial_pose=obj[i].initial_pose
            traj_length=np.linalg.norm(des_pose[0:3]-initial_pose[0:3])
            R_des=PyKDL.Rotation.RPY(des_pose[3],des_pose[4],des_pose[5])
            R_reach=PyKDL.Rotation.RPY(reached_pose[3],reached_pose[4],reached_pose[5])

            
            des=np.array(R_des.GetQuaternion())
            reach=np.array(R_reach.GetQuaternion())
            q_sum=np.linalg.norm(des+reach,ord=2)
            q_diff=np.linalg.norm(des-reach,ord=2)
            
            R_err=100*np.min([q_sum,q_diff])/np.sqrt(2)
            #position error:
            pos_err=100*np.linalg.norm(des_pose[0:3]-reached_pose[0:3],ord=2)/traj_length
            err=[pos_err,R_err]        
            obj[i].err=err
            E_pos.append(pos_err)
        else:
            obj[i].err=[-1.0,-1.0]
    print np.min(E_pos),np.max(E_pos),np.mean(E_pos)
   
    return obj,names

def compute_reached_errors(obj):
    names=[]
    E_pos=[]
    E_R=[]
    # Compute errors:
    for i in range(len(obj)):
        if(len(obj[i].reached)>2):
            des_pose=obj[i].des
            reached_pose=obj[i].reached
            initial_pose=obj[i].initial_pose
            
            pos_length=np.linalg.norm(des_pose[0:3]-initial_pose[0:3],ord=2)
            R_des=PyKDL.Rotation.RPY(des_pose[3],des_pose[4],des_pose[5])
            R_reach=PyKDL.Rotation.RPY(reached_pose[3],reached_pose[4],reached_pose[5])

            
            des=np.array(R_des.GetQuaternion())
            reach=np.array(R_reach.GetQuaternion())
            q_sum=np.linalg.norm(des+reach,ord=2)
            q_diff=np.linalg.norm(des-reach,ord=2)
            
            R_err=100*np.min([q_sum,q_diff])/np.sqrt(2)
            #position error:
            #print obj[i].length
            pos_err=100*np.linalg.norm(des_pose[0:3]-reached_pose[0:3],ord=2)/pos_length
            if(pos_err<30.0):
                names.append(obj[i].f_name+'_'+str(obj[i].trial))
            err=[pos_err,R_err]        
            obj[i].err=err            
            E_pos.append(pos_err)
            E_R.append(R_err)
        else:
            obj[i].err=[-1.0,-1.0]

    #print 100-100*np.median(E_R)/np.sqrt(2)
    print np.median(E_pos)
    return obj,names

def compute_reached_errors_norm(obj):
    names=[]
    E_pos=[]
    E_R=[]
    # Compute errors:
    for i in range(len(obj)):
        if(len(obj[i].reached)>2):
            des_pose=obj[i].des
            reached_pose=obj[i].reached
        
            R_des=PyKDL.Rotation.RPY(des_pose[3],des_pose[4],des_pose[5])
            R_reach=PyKDL.Rotation.RPY(reached_pose[3],reached_pose[4],reached_pose[5])
            
            initial_pose=obj[i].initial_pose
            traj_length=np.linalg.norm(des_pose[0:3]-initial_pose[0:3])
            des=np.array(R_des.GetQuaternion())
            reach=np.array(R_reach.GetQuaternion())
            q_sum=np.linalg.norm(des+reach,ord=2)
            q_diff=np.linalg.norm(des-reach,ord=2)
            
            R_err=100*np.min([q_sum,q_diff])/np.sqrt(2)
            #position error:
            pos_err=100*np.linalg.norm(des_pose[0:3]-reached_pose[0:3],ord=2)/traj_length
            if(pos_err>150.0):
                names.append(obj[i].f_name+'_'+obj[i].trial+'_'+str(des_pose[0:3]-reached_pose[0:3]))
            err=[pos_err,R_err]        
            obj[i].err=err            
            E_pos.append(pos_err)
            E_R.append(R_err)
        else:
            obj[i].err=[-1.0,-1.0]
            #names.append(obj[i].f_name+'_'+obj[i].trial)
    print np.median(E_pos)
    # print np.median(E_R)
    return obj,names

def compute_reached_errors_var(obj):
    names=[]
    E_pos=[]
    E_R=[]
    # Compute errors:
    for i in range(len(obj)):
        if(len(obj[i].reached)>2):
            des_pose=obj[i].des
            reached_pose=obj[i].reached
            
            
            R_des=PyKDL.Rotation.RPY(des_pose[3],des_pose[4],des_pose[5])
            R_reach=PyKDL.Rotation.RPY(reached_pose[3],reached_pose[4],reached_pose[5])

            
            des=np.array(R_des.GetQuaternion())
            reach=np.array(R_reach.GetQuaternion())
            q_sum=np.linalg.norm(des+reach,ord=2)
            q_diff=np.linalg.norm(des-reach,ord=2)
            
            R_err=100*np.min([q_sum,q_diff])/np.sqrt(2)
            #position error:
            pos_err=100*np.linalg.norm(des_pose[0:3]-reached_pose[0:3],ord=2)#/obj[i].length
            if(pos_err>70):
                names.append(obj[i].f_name+'_'+obj[i].trial)
            err=[pos_err,R_err]        
            obj[i].err=[-1.0,-1.0]      
            E_pos.append(pos_err)
            E_R.append(R_err)
            
            if(i%5==4):
                obj[i].err=[np.var(E_pos),np.var(E_R)]      
                if(np.var(E_pos)>1.0):
                    names.append(obj[i].f_name+'_'+obj[i].trial+str(E_pos))
                E_pos=[]
                E_R=[]
        else:
            obj[i].err=[-1.0,-1.0]
            #names.append(obj[i].f_name+'_'+obj[i].trial)
    #print np.median(E_pos)
    # print np.median(E_R)
    return obj,names

def compute_reached_errors_mean(obj):
    names=[]
    n=5
    i=0
    new_obj=[]
    # Compute errors:
    while(i<len(obj)):
        p_err_=[]
        r_err_=[]
        for k in range(5):
            if(obj[k+i].err[0]!=0):
                p_err_.append(obj[k+i].err[0])
                r_err_.append(obj[k+i].err[1])
                
        if(p_err_):
            new_=obj[i]
            new_.err=[np.mean(p_err_),np.mean(r_err_)]
            new_obj.append(new_)
        i=i+5
            

        
    return new_obj

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
    '''
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
    '''
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

    ax=sns.boxplot(x="orientation",y="object",data=data,orient="h",hue='weight',linewidth=1,palette=safeuiall,whis=10.0)
    ax=sns.swarmplot(x='orientation',y='object',data=data,orient="h",hue='weight',linewidth=1,palette=singleui,split=True,edgecolor='black')
    ax.set(ylabel='Objects', xlabel='Orientation Error')
    handles, labels = ax.get_legend_handles_labels()
    
    ax.legend(handles[0:2], labels[0:2],title='bias cost(max. iterations)',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=2, fancybox=False, shadow=False)
    plt.xlim(0)
    #plt.savefig('/home/bala/Dropbox/convergence_exp_1_position_error.png', bbox_inches='tight')
    plt.show()
 
def plot_convergence_method(obj,n,names_=['IK-rigid','PC','relaxed-position','relaxed-full']):
    #Load data into pandas
    p_err=[]
    r_err=[]
    names=[]
    method=[]

    ycb_names={'Lego':'Lego\nDublo',
               'Banana': 'banana',
               'Apple':'apple',
               'Orange': 'orange',
               'Tuna':'tuna\nfish can',
               'Screwdriver':'Phillips\n-head\nscrewdriver',
               'Jello':'gelatin\nbox',
               'Spatula':'spatula',
               'Fork':'fork',
               'Plane':'airplane\ntoy',
               'All objects':'All\nobjects'
    } 
    for i in range(len(obj)):
        if(obj[i].err[0]!=-1.0):
            p_err.append(obj[i].err[0])
            r_err.append(obj[i].err[1])
            
            names.append(obj[i].name)
        
            if(i<n[0]):
                method.append(names_[0])
            if(n[0]<=i and i<(n[0]+n[1])):
                method.append(names_[1])
            elif(n[0]+n[1]<=i and  i<(n[0]+n[1]+n[2])):
              method.append(names_[2])
            elif(n[0]+n[1]+n[2]<=i and  i<(n[0]+n[1]+n[2]+n[3])):
                method.append(names_[3])
    # Adding an additional label for all objects   
    for i in range(len(obj)):
        if(obj[i].err[0]!=-1.0):
            names.append('All objects')
        
    
            if(i<n[0]):
                method.append(names_[0])
            if(n[0]<=i and i<(n[0]+n[1])):
                method.append(names_[1])
            elif(n[0]+n[1]<=i and  i<(n[0]+n[1]+n[2])):
                method.append(names_[2])
            elif(n[0]+n[1]+n[2]<=i and  i<(n[0]+n[1]+n[2]+n[3])):
                method.append(names_[3])

            p_err.append(obj[i].err[0])
            r_err.append(obj[i].err[1])

    # rename the objects:
    for i in range(len(names)):
        for j in range(len(ycb_names.keys())):
            if(names[i]==ycb_names.keys()[j]):
                names[i]=ycb_names[ycb_names.keys()[j]]
    data = pd.DataFrame()
    data['position']=p_err
    data['orientation']=r_err
    data['object']=names
    data['method']=method

    # Plot using seaborn:
    plt.figure(figsize=(9,4))
    
    
    sns.set(font_scale=1)
    sns.set_style("whitegrid")

    ax=sns.boxplot(y="position",x="object",data=data,hue="method",orient="v",linewidth=1,palette=safeuiall,whis=1.5,width=0.7,showfliers=False)
    #limits=np.arange(0, 6, 0.5)
    #limits=np.arange(0, 125, 15)
    #plt.yticks(limits)
    #ax=sns.swarmplot(x='position',y='object',data=data,orient="h",hue="method",linewidth=1,palette=singleui,split=True)
    #handles, labels = ax.get_legend_handles_labels()
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='lower center', bbox_to_anchor=(0.47, -0.35),ncol=5, fancybox=False, shadow=False)
   
    #ax.get_legend().remove()

    #ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Position Error %',xlabel='Objects')
    #plt.ylim(0,5.5)

    '''
    # wrap text in x axis:
    x_lab=ax.get_xticklabels()
    #print x_lab
    x_lab = [ '\n'.join(wrap(l.get_text(), 10)) for l in x_lab ]
    ax.set_xticklabels(x_lab)
    '''
    #plt.ylim(0,210)
    #ax.get_yaxis().set_minor_locator(mpl.ticker.AutoMinorLocator(n=2))
    #ax.grid(b=True, which='minor', color='0.85', linewidth=0.5)
    plt.savefig('position_error_perc.eps', bbox_inches='tight')
    #plt.show()
     # Plot using seaborn:
    plt.figure(figsize=(9,4))
       
    
    sns.set(font_scale=1.)
    sns.set_style("whitegrid")

    ay=sns.boxplot(y="orientation",x="object",data=data,hue="method",orient="v",linewidth=1,palette=safeuiall,whis=1.5,showfliers=False,width=0.7)
    #ay=sns.swarmplot(x='orientation',y='object',data=data,orient="h",hue="method",linewidth=1,palette=singleui,split=True)
    handles, labels = ay.get_legend_handles_labels()
    ay.legend(handles[0:4], labels[0:4],loc='lower center', bbox_to_anchor=(0.47, -0.35),ncol=5, fancybox=False, shadow=False)
    ay.set(ylabel='Orientation Error %',xlabel='Objects')
    plt.yticks(range(0,50,5))
    #plt.ylim(0,100)
    plt.savefig('orientation_error.eps', bbox_inches='tight')
    #plt.show()

        

def plot_convergence_initial(obj,n=0):
    #Load data into pandas
    p_err=[]
    r_err=[]
    names=[]
    method=[]
    
    for i in range(len(obj)):
        if(obj[i].err[0]!=-1.0):
            p_err.append(obj[i].init_err[0])
            r_err.append(obj[i].init_err[1])
            names.append(obj[i].name)
            method.append('Initial Pose')
            p_err.append(obj[i].err[0])
            r_err.append(obj[i].err[1])
            names.append(obj[i].name)
            method.append('Reached Pose')

    # Adding an additional label for all objects   
    for i in range(len(obj)):
        if(obj[i].err[0]!=-1.0):
            p_err.append(obj[i].init_err[0])
            r_err.append(obj[i].init_err[1])
            names.append('All objects')

            method.append('Initial Pose')
            p_err.append(obj[i].err[0])
            r_err.append(obj[i].err[1])
            names.append('All objects')

            method.append('Reached Pose')

    data = pd.DataFrame()
    data['position']=p_err
    data['orientation']=r_err
    data['object']=names
    data['method']=method

    # Plot using seaborn:
    plt.figure(figsize=(4,4))
       
    
    sns.set(font_scale=1.)
    sns.set_style("whitegrid")

    ax=sns.boxplot(y="position",x="object",data=data,hue="method",orient="v",linewidth=1,palette=safeuiall,whis=10.0)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Position Error (cm)', xlabel='Objects')
        
    plt.savefig('random_position_error.eps', bbox_inches='tight')
    # Plot using seaborn:
    plt.figure(figsize=(4,4))
       
    
    sns.set(font_scale=1.)
    sns.set_style("whitegrid")

    ax=sns.boxplot(y="orientation",x="object",data=data,hue="method",orient="v",linewidth=1,palette=safeuiall,whis=10.0)
    #ax=sns.swarmplot(x='position',y='object',data=data,orient="h",hue="method",linewidth=1,palette=singleui,split=True)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Orientation Error %', xlabel='Objects')
    #plt.xlim(0,100)
        
    plt.savefig('random_orientation_error.eps', bbox_inches='tight')
    #plt.show()

        
    

def plot_single():
    obj_3=TrajList("../data/trials/temp.csv",folder='')
    print len(obj_3.arr) 
    traj_n=125
    obj,n=compute_errors(obj_3.arr)
    plot_convergence_method(obj,traj_n) 
   
    
    

def plot_random_method():
    obj_0=TrajList('../data/trajectories/random_position_solution.csv','')
    obj_1=TrajList('../data/trajectories/random_full_solution.csv','')

    print len(obj_0.arr)
    traj_n=len(obj_0.arr)
    obj_all=obj_0.arr+obj_1.arr#+obj_2.arr+obj_3.arr
    obj,n=compute_errors(obj_all)

    plot_convergence_method(obj,traj_n)

def plot_method():

    #obj_0=TrialList('../data/trials/ik-rigid.csv','',5)
    obj_0=TrajList('../data/trajectories/ik-rigid_solution.csv','').arr
    #obj_0=obj_0.get_traj_list(5)
    obj_1=TrialList('../data/trials/relaxed-position.csv','',5)
    obj_1=obj_1.get_traj_list(5)

    obj_2=TrialList('../data/trials/relaxed-position-orientation.csv','',5) 
    obj_2=obj_2.get_traj_list(5)

    obj_3=TrialList("../data/trials/relaxed-rigidity.csv",'',5)
    obj_3=obj_3.get_traj_list(5)
    #traj_n=len(obj_1)
    #obj_all=obj_0+obj_1+obj_2+obj_3

    obj_0,n=compute_errors(obj_0)
    exit()
    #print names_[0]+': '+str(100-100*len(n)/500.0)
    #obj_1=compute_reached_errors_mean(obj_1)
    obj_1,n=compute_errors(obj_1)
    #print names_[1]+': '+str(100-100*len(n)/500.0)

    #obj_2=compute_reached_errors_mean(obj_2)
    obj_2,n=compute_errors(obj_2)
    #print names_[2]+': '+str(100-100*len(n)/500.0)

    #obj_3=compute_reached_errors_mean(obj_3)
    obj_3,n=compute_errors(obj_3)
    #print names_[3]+': '+str(100-100*len(n)/500.0)
    #print n
    obj=obj_0+obj_1+obj_2+obj_3

    #print len(obj_all)
    #obj,n=compute_errors(obj_all)
    exit()
    plot_convergence_method(obj,[len(obj_0),len(obj_1),len(obj_2),len(obj_3)],names_=['IK-rigid','relaxed-position','relaxed-position-orientation','relaxed-rigidity'])

def plot_reached():
    root_dir = '../../in_hand_manipulation_data/data/in_grasp_manipulation'
    obj_1=TrialList(root_dir+'/trials/trinkle.csv','',5)
    obj_2=TrialList(root_dir+'/trials/relaxed-position.csv','',5)

    obj_3=TrialList(root_dir+'/trials/relaxed-position-orientation.csv','',5)
    obj_4=TrialList(root_dir+'/trials/relaxed-rigidity.csv','',5)

    names_=['point contact','relaxed-position','relaxed-position-orientation','relaxed-rigidity']
    #obj_all=obj_1.arr+obj_2.arr+obj_3.arr#+obj_4.arr
    obj_1,n=compute_reached_errors(obj_1.arr)
    #print names_[0]+': '+str(100-100*len(n)/500.0)
    #obj_1=compute_reached_errors_mean(obj_1)
    obj_2,n=compute_reached_errors(obj_2.arr)
    #print names_[1]+': '+str(100-100*len(n)/500.0)

    #obj_2=compute_reached_errors_mean(obj_2)
    obj_3,n=compute_reached_errors(obj_3.arr)
    #print names_[2]+': '+str(100-100*len(n)/500.0)

    #obj_3=compute_reached_errors_mean(obj_3)
    obj_4,n=compute_reached_errors(obj_4.arr)
    #print names_[3]+': '+str(100-100*len(n)/500.0)
    #print n
    obj=obj_1+obj_2+obj_3+obj_4
    #plot_convergence_method(obj,[len(obj_1),len(obj_2),len(obj_3),len(obj_4)],names_=['trinkle','relaxed-position','relaxed-full','relaxed-full-linear'])
    plot_convergence_method(obj,[len(obj_1),len(obj_2),len(obj_3),len(obj_4)],names_)
def plot_reached_norm():
    obj_1=TrialList('../data/trials/trinkle_traj_length.csv','',5)
    obj_2=TrialList('../data/trials/relaxed-position_traj_length.csv','',5)
    obj_3=TrialList('../data/trials/relaxed-position-orientation.csv','',5)
    obj_4=TrialList("../data/trials/relaxed-rigidity.csv",'',5)

    names_=['Point Contact','relaxed-position','relaxed-position-orientation','relaxed-rigidity']
    #obj_all=obj_1.arr+obj_2.arr+obj_3.arr#+obj_4.arr
    obj_1,n=compute_reached_errors_norm(obj_1.arr)
    print names_[0]+': '+str(100-100*len(n)/500.0)
    #obj_1=compute_reached_errors_mean(obj_1)
    obj_2,n=compute_reached_errors_norm(obj_2.arr)
    print names_[1]+': '+str(100-100*len(n)/500.0)

    #obj_2=compute_reached_errors_mean(obj_2)
    obj_3,n=compute_reached_errors_norm(obj_3.arr)
    print names_[2]+': '+str(100-100*len(n)/500.0)
    print n
    #obj_3=compute_reached_errors_mean(obj_3)
    obj_4,n=compute_reached_errors_norm(obj_4.arr)
    print names_[3]+': '+str(100-100*len(n)/500.0)

    obj=obj_1+obj_2+obj_3+obj_4
    #plot_convergence_method(obj,[len(obj_1),len(obj_2),len(obj_3),len(obj_4)],names_=['trinkle','relaxed-position','relaxed-full','relaxed-full-linear'])
    plot_convergence_method(obj,[len(obj_1),len(obj_2),len(obj_3),len(obj_4)],names_)

def plot_reached_variance():
    obj_1=TrialList('../data/trials/trinkle_traj_length.csv','',5)
    obj_2=TrialList('../data/trials/relaxed-position_traj_length.csv','',5)

    obj_3=TrialList('../data/trials/relaxed-full_traj_length.csv','',5)
    obj_4=TrialList("../data/trials/relaxed-full-linear.csv",'',5)

    names_=['Point Contact','relaxed-position','relaxed-position-orientation','relaxed-rigidity']
    #obj_all=obj_1.arr+obj_2.arr+obj_3.arr#+obj_4.arr
    obj_1,n=compute_reached_errors_var(obj_1.arr)
    print names_[0]+': '+str(100-100*len(n)/500.0)
    #obj_1=compute_reached_errors_mean(obj_1)
    obj_2,n=compute_reached_errors_var(obj_2.arr)
    print names_[1]+': '+str(100-100*len(n)/500.0)

    #obj_2=compute_reached_errors_mean(obj_2)
    obj_3,n=compute_reached_errors_var(obj_3.arr)
    print names_[2]+': '+str(100-100*len(n)/500.0)
    print n
    #obj_3=compute_reached_errors_mean(obj_3)
    obj_4,n=compute_reached_errors_var(obj_4.arr)
    print names_[3]+': '+str(100-100*len(n)/500.0)
    obj=obj_1+obj_2+obj_3+obj_4
    #plot_convergence_method(obj,[len(obj_1),len(obj_2),len(obj_3),len(obj_4)],names_=['trinkle','relaxed-position','relaxed-full','relaxed-full-linear'])
    plot_convergence_method(obj,[len(obj_1),len(obj_2),len(obj_3),len(obj_4)],names_)

def plot_initial():
    
    obj_0=TrialList('../data/trials/relaxed-full-linearrandom_traj_length.csv','',5)
    obj_all=obj_0.arr
    obj,n=compute_reached_errors(obj_all)
    #print n
    obj,n=compute_initial_errors(obj_all)

    plot_convergence_initial(obj)

def compute_human_initial_err(obj):
    names=[]
    P_err=[]
    O_err=[]
    perc_P_err=[]
    n = 5 # number of trials:
    initial_pose=np.array(obj[0].initial_pose)
    for i in range(len(obj)):
        if( i % 5 == 0):
            initial_pose=np.array(obj[i].exp_pose)
        else:
            des_pose=np.array(obj[i].des)
            exp_pose=np.array(obj[i].exp_pose)
            #initial_pose=np.array(obj[i].initial_pose)
            #reached_pose=np.array(obj[i].reached)
            
            R_des=PyKDL.Rotation.RPY(initial_pose[3],initial_pose[4],initial_pose[5])
            R_reach=PyKDL.Rotation.RPY(exp_pose[3],exp_pose[4],exp_pose[5])

            
            des=np.array(R_des.GetQuaternion())
            reach=np.array(R_reach.GetQuaternion())
            q_sum=np.linalg.norm(des+reach,ord=2)
            q_diff=np.linalg.norm(des-reach,ord=2)
            
            R_err=100*np.min([q_sum,q_diff])/np.sqrt(2)
            #position error:
            pos_err=100*np.linalg.norm(initial_pose[0:3]-exp_pose[0:3],ord=2)
            pos_perc_err=pos_err/np.linalg.norm(des_pose[0:3]-initial_pose[0:3],ord=2)

            names.append(obj[i].f_name+'_'+str(obj[i].trial))#+str(pos_err))
            obj[i].err=[pos_err,R_err]
            #if(R_err < 10.0):
            P_err.append(pos_err)
            perc_P_err.append(pos_perc_err)
            O_err.append(R_err)
    print len(P_err)
    print "Position error cm : ",np.median(P_err),np.var(P_err)#, np.min(P_err)
    #print "Position error % : ",np.mean(perc_P_err),np.max(perc_P_err), np.min(perc_P_err)
        
    print "Ori error: ",np.median(O_err),np.var(O_err)#, np.min(O_err)

    return obj,names

            

def compute_robust_err(obj,n,initial_=False):
    names=[]
    P_err=[]
    for i in range(len(obj)):
        if(int(obj[i].trial)==0):
            des_pose=np.array(obj[i].reached)
            if(initial_):
                des_pose=np.array(obj[i].exp_pose)
            obj[i].err=[-1.0,-1.0]
        else:
            reached_pose=np.array(obj[i].reached)
            if(initial_):
                reached_pose=np.array(obj[i].exp_pose)

            R_des=PyKDL.Rotation.RPY(des_pose[3],des_pose[4],des_pose[5])
            R_reach=PyKDL.Rotation.RPY(reached_pose[3],reached_pose[4],reached_pose[5])

            
            des=np.array(R_des.GetQuaternion())
            reach=np.array(R_reach.GetQuaternion())
            q_sum=np.linalg.norm(des+reach,ord=2)
            q_diff=np.linalg.norm(des-reach,ord=2)
            
            R_err=100*np.min([q_sum,q_diff])/np.sqrt(2)
            #position error:
            pos_err=100*np.linalg.norm(des_pose[0:3]-reached_pose[0:3],ord=2)
            if(initial_==True and pos_err>1.1):
                names.append(obj[i].f_name+'_'+str(obj[i].trial))#+str(pos_err))
            obj[i].err=[pos_err,R_err]
            P_err.append(pos_err)
    print np.mean(P_err)
    print np.var(P_err)
    return obj,names

            


def plot_robust():
    obj_0=TrialList("../data/trials/relaxed-full-linear_init.csv",'',5).arr
    obj_1=TrialList("../data/trials/relaxed-full-linear_init.csv",'',5).arr
    n_trials=5
    obj_reached,n=compute_robust_err(obj_0,n_trials)
    obj_init,n=compute_robust_err(obj_1,n_trials,initial_=True)
    plot_convergence_method(obj_init+obj_reached,[len(obj_init),len(obj_reached),len(obj_reached),len(obj_reached)],['Initial Pose','Reached pose'])
    

def get_initial_err():
    root_dir = '../../in_hand_manipulation_data/data/in_grasp_manipulation'

    obj_0=TrialList(root_dir+'/trials/trinkle_init.csv','',5).arr
    compute_human_initial_err(obj_0)

def plot_pair():
    #obj_0=TrialList("../data/trials/relaxed-full-linear_init.csv",'',5).arr
    #obj_1=TrialList("../data/trials/relaxed-full-linear_init.csv",'',5).arr
    obj_0=TrialList("../data/trials/trinkle_init.csv",'',5).arr
    obj_1=TrialList("../data/trials/trinkle_init.csv",'',5).arr
    n_trials=5
    #obj_reached,n=compute_robust_err(obj_0,n_trials)
    obj_init,n=compute_robust_err(obj_1,n_trials,initial_=True)
    '''
    print n
    p_err=[]
    r_err=[]
    names=[]
    method=[]
    traj=[]
    #plt.figure(figsize=(4,4))
    sns.set(font_scale=1,rc={"lines.linewidth": 1.0})
    sns.set_style("whitegrid")
    for i in range(len(obj_0)):

        if(obj_reached[i].err[0]!=-1.0):
            p_err.append(obj_init[i].err[0])
            r_err.append(obj_init[i].err[1])
            if(i/5%10<5):
                traj.append('PC:Grasp-1')       
                traj.append('PC:Grasp-1')       
            else:
                traj.append('PC:Grasp-2')       
                traj.append('PC:Grasp-2')       
            names.append(obj_init[i].name)
            method.append('Initial')
            p_err.append(obj_reached[i].err[0])
            r_err.append(obj_reached[i].err[1])
            names.append(obj_reached[i].name)

            method.append('Reached')

    obj_0=TrialList("../data/trials/relaxed-full-linear_init.csv",'',5).arr
    obj_1=TrialList("../data/trials/relaxed-full-linear_init.csv",'',5).arr
    n_trials=5
    obj_reached,n=compute_robust_err(obj_0,n_trials)
    obj_init,n=compute_robust_err(obj_1,n_trials,initial_=True)

    for i in range(len(obj_0)):
        if(obj_reached[i].err[0]!=-1.0):
            p_err.append(obj_init[i].err[0])
            r_err.append(obj_init[i].err[1])
            if(i/5%10<5):
                traj.append('Relaxed-Rigidity:Grasp-1')       
                traj.append('Relaxed-Rigidity:Grasp-1')       
            else:
                traj.append('Relaxed-Rigidity:Grasp-2')       
                traj.append('Relaxed-Rigidity:Grasp-2')       
            names.append(obj_init[i].name)
            method.append('Initial')
            p_err.append(obj_reached[i].err[0])
            r_err.append(obj_reached[i].err[1])
            names.append(obj_reached[i].name)

            method.append('Reached')


    #plt.savefig('robust_position_error_apple.eps', bbox_inches='tight')

    data = pd.DataFrame()
    data['position']=p_err
    data['orientation']=r_err
    data['Object']=names
    data['Pose']=method
    data['traj']=traj
    #ax=sns.PairGrid(data=data,y_vars="position",x_vars=['method'],hue='traj')
    #ax.map(sns.pointplot,dodge=True)
    #ax=sns.PairGrid(data=data)
    ax = sns.factorplot(data=data,y='position', x='Pose',hue='traj',dodge=True, sharey=False,col='Object',size=3,aspect=0.6,col_wrap=5,legend=False,palette=fourui,ci=100)
    ax.set_axis_labels("Pose", "Position (cm)")
    ax.set_titles("{col_name}")
    handles = ax._legend_data.values()
    labels = ax._legend_data.keys()
    #ax.add_legend(ncol=5)
    ax.fig.legend(handles, labels,loc='upper center', bbox_to_anchor=(0.5, 1.02),ncol=5, fancybox=False, shadow=False)
    #handles, labels = ax.get_legend_handles_labels()
    #ax.set_xticklabels(["Men", "Women", "Children"])
    #ax.set_titles("{col_name} {col_var}")
    #handles, labels = ax.get_legend_handles_labels()
    plt.ylim(0,)
    
    #ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    #ax.set(ylabel='Position Error (cm)', xlabel='')

    
    #ax=sns.PairGrid(data=data,y_vars="position",x_vars=["Object","method"],hue="object",size=5, aspect=.5)
    #ax = sns.pointplot(y="position", x="method", data=data,ci=None)
    #plt.savefig('robust_error.eps', bbox_inches='tight')
   
    #plt.show()
    '''
def plot_regression():
    obj_0=TrialList("../data/trials/relaxed-full-linear_init.csv",'',5).arr
    obj_1=TrialList("../data/trials/relaxed-full-linear_init.csv",'',5).arr
    n_trials=5
    obj_reached,n=compute_robust_err(obj_0,n_trials)
    obj_init,n=compute_robust_err(obj_1,n_trials,initial_=True)
    x=[]
    y=[]
    x_r=[]
    y_r=[]
    color='red'
    names=[]
    for i in range(len(obj_init)):
        if(obj_init[i].err[0]!=-1.0):
            plt.plot([0.0,1.0],[obj_init[i].err[0],obj_reached[i].err[0]], '-o')
            x.append(obj_init[i].err[0])
            names.append(0.0)
            y.append(obj_reached[i].err[0])

    #plotData(x,y,'red')

    plt.show()
    '''

    ax=sns.regplot(np.array(x),np.array(y),robust=True, ci=None)
    plt.ylim(0,)
    plt.xlim(0,)
    plt.show()
    '''
if __name__=='__main__':
    '''
    method_arr=["relaxed-rigidity","trinkle",'relaxed-position','relaxed-position-orientation']
    #method_arr=["2f_4f_relaxed-rigidity","random_relaxed-rigidity"]
    for method in method_arr:

        obj_0=TrialList("../data/trials/"+method+".csv",'',5)
        obj_1=obj_0.get_traj_list(5)
        obj_0.write_traj_list("../data/trajectories/"+method+".csv",obj_1)
    '''
    #plot_single()
    #plot_initial()
    #plot_method()
    #plot_reached()#_norm()
    #plot_reached_variance()
    #plot_robust()
    #plot_regression()
    #plot_pair()
    #plot_4f()

    get_initial_err()
