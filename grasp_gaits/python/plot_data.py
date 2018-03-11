
import rospy

from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped,PointStamped
import numpy as np
import sys
import time
import pickle
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from collections import OrderedDict
import pandas as pd
sns.set(style="whitegrid")
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
#plt.style.use('')
#twoui=['#a6611a','#dfc27d','#f5f5f5','#80cdc1','#018571']
safeuiall=['#e41a1c','#4daf4a','#984ea3','#ff7f00','#377eb8']
ui=['#ffffff','#b2abd2','#5e3c99','#000000','#fdb863']
ui_2=['#d7191c','#fdae61','#ffffbf','#abd9e9','#2c7bb6','#000000']
ui_3=['#e41a1c','#377eb8','#4daf4a','#984ea3','#ff7f00','#000000']


def plot_convergence_all(obj_names_sd, obj_names_svd, obj_idx):
    colors =safeuiall
    plt.figure(figsize=(4,4))
    sns.set(font_scale=1.25)
    sns.set_style("whitegrid")
    d_arr=[]
    max_len=-1
    for o in range(len(obj_names_sd)):
        d=np.zeros(100)
        o_name=obj_names_sd[o]
        file_ = open(o_name,'rb')
        object_file = pickle.load(file_)
        file_.close()
        G_arr=object_file

        x=[]
        G=G_arr[0]
        # Compute error:
        c_pts=G[3]
        d_pts=G[2]
        diff=np.zeros(4)
        for i in range(len(d_pts)):
            diff[i]=np.sqrt(((d_pts[i].point.x-c_pts[i].point.x)**2+
                             (d_pts[i].point.y-c_pts[i].point.y)**2+(d_pts[i].point.z-c_pts[i].point.z)**2))

        init_diff=np.sum(diff)

        for k in range(1,len(G_arr)-2):
            G=G_arr[k]
            # Compute error:
            
            if(not isinstance(G,int)):
                c_pts=G[3]
                d_pts=G[2]
                diff=np.zeros(4)
                for i in range(len(d_pts)):
                    diff[i]=np.sqrt(((d_pts[i].point.x-c_pts[i].point.x)**2+
                                     (d_pts[i].point.y-c_pts[i].point.y)**2+(d_pts[i].point.z-c_pts[i].point.z)**2))

                diff_sum=np.sum(diff)/init_diff
                d[k-1]=diff_sum
        if((len(G_arr)-2)>max_len):
            max_len=len(G_arr)-2

        d_arr.append(d)
    d_sum=np.sum(d_arr,axis=0)/25.0
    plt.errorbar(y=d_sum[0:49],x=range(49),label='SD',color=colors[1],linestyle='--',linewidth=3)
    d_arr=[]
    max_len=-1
    for o in range(len(obj_names_svd)):
        d=np.zeros(100)

        o_name=obj_names_svd[o]
        file_ = open(o_name,'rb')
        object_file = pickle.load(file_)
        file_.close()
        G_arr=object_file

        x=[]
        G=G_arr[0]
        # Compute error:
        c_pts=G[3]
        d_pts=G[2]
        diff=np.zeros(4)
        for i in range(len(d_pts)):
            diff[i]=np.sqrt(((d_pts[i].point.x-c_pts[i].point.x)**2+
                             (d_pts[i].point.y-c_pts[i].point.y)**2+(d_pts[i].point.z-c_pts[i].point.z)**2))

        init_diff=np.sum(diff)

        for k in range(1,len(G_arr)-2):
            G=G_arr[k]
            # Compute error:
            c_pts=G[3]
            d_pts=G[2]
            diff=np.zeros(4)
            for i in range(len(d_pts)):
                diff[i]=np.sqrt(((d_pts[i].point.x-c_pts[i].point.x)**2+
                                 (d_pts[i].point.y-c_pts[i].point.y)**2+(d_pts[i].point.z-c_pts[i].point.z)**2))

            diff_sum=np.sum(diff)/init_diff
            d[k-1]=diff_sum

        if((len(G_arr)-2)>max_len):
            max_len=len(G_arr)-2
        #plt.plot(d,label=obj_idx[int(o/5)],color=colors[int(o/5)])
        d_arr.append(d)

    d_sum=np.sum(d_arr,axis=0)/25.0
    e_=np.std(d_arr,axis=0)
    plt.errorbar(y=d_sum[0:49],x=range(49),label='SVD',color=colors[0],linewidth=3)


    
    #plt.plot(d_sum_sd,label='SD',color=colors[0])
    d=np.ones(100)*1e-1
    #plt.plot(d,label='minimum error',color=colors[-1],linestyle='-.')
    plt.ylabel('Normalized Average Point Error')
    plt.xlabel('Iteration')
    plt.ylim(ymin=0)
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = OrderedDict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(),loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    plt.savefig('convergence.eps', bbox_inches='tight')
    #plt.show()

def plot_convergence(obj_names,obj_idx):
    colors =ui_3
    plt.figure(figsize=(6,4))
    sns.set(font_scale=1.45)
    #sns.set_style("whitegrid")
        
    for o in range(len(obj_names)):
        o_name=obj_names[o]
        file_ = open(o_name,'rb')
        object_file = pickle.load(file_)
        file_.close()
        G_arr=object_file

        d=[]
        x=[]
        for i in range(0,len(G_arr)-1):
            G=G_arr[i]
            # Compute error:
            c_pts=G[3]
            d_pts=G[2]
            diff=np.zeros(4)
            for i in range(len(d_pts)):
                diff[i]=np.sqrt(((d_pts[i].point.x-c_pts[i].point.x)**2+
                                 (d_pts[i].point.y-c_pts[i].point.y)**2+(d_pts[i].point.z-c_pts[i].point.z)**2))

                diff_sum=np.sum(diff)
            d.append(diff_sum)                
        plt.plot(d,label=obj_idx[int(o/5)],color=colors[int(o/5)])
        print int(o/5)
    d=[]
    for i in range(100):
        d.append(1e-2)
    plt.plot(d,label='minimum error',color=colors[-1],linestyle='-.')
    plt.ylabel('Sum of errors in reaching contact points(m)')
    plt.xlabel('Iteration')
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = OrderedDict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(),loc='upper center', bbox_to_anchor=(0.5, 1.07),ncol=5, fancybox=False, shadow=False)

    plt.show()

def plot_ctime(obj_names,obj_idx):
    colors =ui_3

    sns.set(font_scale=1.45)
    #sns.set_style("whitegrid")
    c_t=[]
    o_names=[]
    for o in range(len(obj_names)):
        o_name=obj_names[o]
        file_ = open(o_name,'rb')
        object_file = pickle.load(file_)
        file_.close()
        G_arr=object_file
        c_t.append(G_arr[-1][1])
        o_names.append(obj_idx[int(o/5)])
    data = pd.DataFrame()
    data['c_time']=c_t
    data['object']=o_names

    plt.figure(figsize=(9,4))
    
    
    sns.set(font_scale=1)
    sns.set_style("whitegrid")

    ax=sns.swarmplot( y="c_time", x="object",data=data, orient="v", linewidth=1,palette=safeuiall)
    plt.ylim(ymin=0)
        
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Planning Time (sec)',xlabel='Objects')
    
    plt.show()

def plot_final_error(obj_names,obj_idx):
    colors =ui_3

    sns.set(font_scale=1.45)
    #sns.set_style("whitegrid")
    c_t=[]
    o_names=[]
    for o in range(len(obj_names)):
        o_name=obj_names[o]
        file_ = open(o_name,'rb')
        object_file = pickle.load(file_)
        file_.close()
        G_arr=object_file
        c_t.append(np.sum(G_arr[-1][2]))
        o_names.append(obj_idx[int(o/5)])
    data = pd.DataFrame()
    data['c_time']=c_t
    data['object']=o_names

    plt.figure(figsize=(9,4))
    
    
    sns.set(font_scale=1)
    sns.set_style("whitegrid")

    ax=sns.swarmplot( y="c_time", x="object",data=data, orient="v", linewidth=1,palette=safeuiall)
    plt.ylim(ymin=0)
        
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Position error (m)',xlabel='Objects')
    
    plt.show()


def get_panda_base(obj_names,obj_idx,label):
    c_t=[]
    m_iter=[]
    avg_err=[]
    max_err=[]
    o_names=[]
    n_opt=[]
    l_=[]
    for o in range(len(obj_names)):
        o_name=obj_names[o]
        file_ = open(o_name,'rb')
        object_file = pickle.load(file_)
        file_.close()
        G_arr=object_file
        m_iter.append(G_arr[-1][0])
        n_opt.append(len(G_arr)-2)
        c_t.append(G_arr[-1][1])
        avg_err.append(np.mean(G_arr[-1][2])*100.0)
        max_err.append(np.max(G_arr[-1][2]))
        o_names.append(obj_idx[int(o/5)])
        l_.append(label)
    data = pd.DataFrame()
    data['c_time']=c_t
    data['object']=o_names
    data['method']=l_
    data['max_iter']=m_iter
    data['max_err']=max_err
    data['avg_err']=avg_err
    data['opt']=n_opt
    return data

def plot_panda(data):
    plt.figure(figsize=(7,4))
    sns.set(font_scale=1.25)
    sns.set_style("whitegrid")
    '''
    ax=sns.swarmplot( y="max_iter", x="object",hue="method",data=data, orient="v", linewidth=1, palette=ui, split=True)
    plt.ylim(ymin=0)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Number of Iterations',xlabel='Objects')
    plt.savefig('iter_plot.eps', bbox_inches='tight')

    
    del ax
    '''
    plt.figure(figsize=(6,4))
    sns.set(font_scale=1.2)
    sns.set_style("whitegrid")

    ax=sns.boxplot( y="opt", x="object",hue="method",data=data, orient="v", linewidth=1, palette=ui, whis=1000)
    plt.ylim(ymin=0,ymax=55)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Number of steps',xlabel='Objects')
    plt.savefig('opt_plot_box.eps', bbox_inches='tight')
   
    del ax
    '''
    plt.figure(figsize=(6,4))
    sns.set(font_scale=1.2)
    sns.set_style("whitegrid")

    ax=sns.swarmplot( y="c_time", x="object",hue="method",data=data, orient="v", linewidth=1, palette=ui, split=True)
    plt.ylim(ymin=0)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Planning time (sec.)',xlabel='Objects')
    plt.savefig('ctime_plot.eps', bbox_inches='tight')
    del ax
    plt.figure(figsize=(6,4))
    sns.set(font_scale=1.2)
    sns.set_style("whitegrid")
    
    ax=sns.swarmplot( y="max_err", x="object",hue="method",data=data, orient="v", linewidth=1, palette=ui, split=True)
    plt.ylim(ymin=0)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Maximum point error(m)',xlabel='Objects')
    plt.savefig('max_err_plot.eps', bbox_inches='tight')
    del ax
    '''
    plt.figure(figsize=(6,4))
    sns.set(font_scale=1.2)
    sns.set_style("whitegrid")

    ax=sns.boxplot( y="avg_err", x="object",hue="method",data=data, orient="v", linewidth=1, palette=ui,whis=100)
    plt.ylim(ymin=0)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Average point error(cm)',xlabel='Objects')
    #plt.savefig('avg_err_plot.eps', bbox_inches='tight')

    plt.show()
    
def plot_final_iter(obj_names,obj_idx):
    colors =ui_3

    sns.set(font_scale=1.45)
    #sns.set_style("whitegrid")
    c_t=[]
    o_names=[]
    for o in range(len(obj_names)):
        o_name=obj_names[o]
        file_ = open(o_name,'rb')
        object_file = pickle.load(file_)
        file_.close()
        G_arr=object_file
        c_t.append(G_arr[-1][0])
        o_names.append(obj_idx[int(o/5)])
    data = pd.DataFrame()
    data['c_time']=c_t
    data['object']=o_names

    plt.figure(figsize=(9,4))
    
    
    sns.set(font_scale=1)
    sns.set_style("whitegrid")

    ax=sns.swarmplot( y="c_time", x="object",data=data, orient="v", linewidth=1,palette=safeuiall)
    plt.ylim(ymin=0)
        
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[0:4], labels[0:4],loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=5, fancybox=False, shadow=False)
    ax.set(ylabel='Number of Iterations',xlabel='Objects')
    
    plt.show()

def plot_single_convergence():
    # load grasp data:
    file_ = open(sys.argv[1],'rb')
    object_file = pickle.load(file_)
    file_.close()
    G_arr=object_file

    
    d=[]
    for i in range(0,len(G_arr)-1):
        G=G_arr[i]
        # Compute error:
        c_pts=G[3]
        d_pts=G[2]
        diff=np.zeros(4)
        for i in range(len(d_pts)):
            diff[i]=np.sqrt(((d_pts[i].point.x-c_pts[i].point.x)**2+
                     (d_pts[i].point.y-c_pts[i].point.y)**2+(d_pts[i].point.z-c_pts[i].point.z)**2))

        diff_sum=np.sum(diff)
        d.append(diff_sum)
    #print d
    plt.plot(d)
    plt.ylabel('Sum of errors in reaching contact points(m)')
    plt.xlabel('Iteration')
    plt.title('Banana')
    plt.show()

def o_files(o_name,l,prefix='r_'):
    ss_names=[]
    for i in range(l):
        ss_names.append(prefix+o_name+'_'+str(i)+'.p')
    return ss_names


if __name__=='__main__':
    l=5
    b_names=o_files('banana',l)
    s_names=o_files('sugar_box',l)
    ss_names=o_files('soft_scrub',l)
    m_names=o_files('mustard',l)
    p_names=o_files('pringles',l)
    names_sd=b_names+s_names+ss_names+m_names+p_names

    b_svd_names=o_files('banana',l,'svd_r_')
    s_svd_names=o_files('sugar_box',l,'svd_r_')
    ss_svd_names=o_files('soft_scrub',l,'svd_r_')
    m_svd_names=o_files('mustard_bottle',l,'svd_r_')
    p_svd_names=o_files('pringles',l,'svd_r_')
    names_svd=b_svd_names+s_svd_names+ss_svd_names+m_svd_names+p_svd_names

    
    #print names
    obj_idx=['banana','sugar_box','soft_scrub','mustard','pringles']

    sd_pd=get_panda_base(names_sd,obj_idx,'sd')
    svd_pd=get_panda_base(names_svd,obj_idx,'svd')
    dat=sd_pd.append(svd_pd)
    #plot_panda(dat)
    plot_convergence_all(names_sd,names_svd,obj_idx)
    
    '''
    o_name=obj_idx[2]
    o_data=sd_pd
    print '**************'
    print o_name
    print 'Ctime'
    print np.max(o_data['c_time']), np.median(o_data['c_time']), np.var(o_data['c_time']) 

    print 'Optimization:'
    print  np.max(o_data['opt']), np.median(o_data['opt']), np.var(o_data['opt'])


    print 'avg err:'
    print np.max(o_data['avg_err']), np.median(o_data['avg_err']), np.var(o_data['avg_err'])

    print "**************SVD"
    o_data=svd_pd
    print 'Ctime'
    print np.max(o_data['c_time']), np.median(o_data['c_time']), np.var(o_data['c_time']) 

    
    print 'Optimization:'
    print  np.max(o_data['opt']), np.median(o_data['opt']), np.var(o_data['opt'])


    print 'avg err:'
    print np.max(o_data['avg_err']), np.median(o_data['avg_err']), np.var(o_data['avg_err'])
    '''
    
    
    '''
    dat=sd_pd.append(svd_pd)

    
    plot_convergence_all(names_sd,names_svd,obj_idx)
    #plot_final_error(names,obj_idx)
    #plot_final_iter(names,obj_idx)
    #plot_single_convergence()

    '''
