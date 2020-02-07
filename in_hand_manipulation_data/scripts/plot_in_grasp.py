import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42
safeuiall = ['#ffffff','#fef0d9','#fdcc8a','#fc8d59','#d7301f']
safeui3 = ['#fef0d9','#fdcc8a','#fc8d59','#d7301f']
twoui=['#fef0d9','#d7301f']
from data_api import pdAPI
from error_api import errorAPI
import seaborn as sns
import numpy as np
sns.set(font_scale=1.45)
sns.set_style("whitegrid")
import pandas as pd
def fb_plots(f_name):
    pd_api=pdAPI()
    df=pd_api.read_pd('../data/in_grasp_manipulation/'+f_name)
    err_api=errorAPI()
    err_p=[]
    err_o=[]
    meth_arr=[]
    obj_arr=[]
    for i in range(len(df['Object'])):
        txt=df['Desired Pose'][i][1:-1]
        des_pose=np.fromstring(txt,dtype=float,sep=' ')
        txt=df['AR Reached Pose'][i][1:-1]
        r_pose=np.fromstring(txt,dtype=float,sep=' ')
        p_err,or_err=err_api.pose_err(des_pose,r_pose)
        err_p.append(p_err*100.0)
        err_o.append(or_err*100.0)
        meth_arr.append(df['method'][i])
        obj_arr.append(df['Object'][i])
    
    for i in range(len(df['Object'])):
        txt=df['Desired Pose'][i][1:-1]
        des_pose=np.fromstring(txt,dtype=float,sep=' ')
        txt=df['AR Reached Pose'][i][1:-1]
        r_pose=np.fromstring(txt,dtype=float,sep=' ')
        p_err,or_err=err_api.pose_err(des_pose,r_pose)
        # adding to all objects:
        obj_arr.append('All Objects')
        err_p.append(p_err*100.0)
        err_o.append(or_err*100.0)
        meth_arr.append(df['method'][i])

    # create a new df:
    df_new=pd.DataFrame()
    df_new['Object']=obj_arr
    df_new['p_err']=err_p
    df_new['o_err']=err_o
    df_new['method']=meth_arr
    #df_new=df.assign(p_err=err_p,o_err=err_o,)
    #order=['ik_obj','fb-ctrl']
    order=['ik_obj','ik_j','fb-ctrl','j_acc_fb']
    if(False):
        p_ik_err=[]
        o_ik_err=[]
        p_fb_err=[]
        o_fb_err=[]
        for i in range(len(df_new['Object'])):
            if(df_new['Object'][i]=='All Objects' and df_new['method'][i]==order[0]):
                p_ik_err.append(df_new['p_err'][i])
                o_ik_err.append(df_new['o_err'][i])
            if(df_new['Object'][i]=='All Objects' and df_new['method'][i]==order[1]):
                p_fb_err.append(df_new['p_err'][i])
                o_fb_err.append(df_new['o_err'][i])
        print 'Open loop error:'
        print np.median(p_ik_err)
        print np.median(o_ik_err)
        print 'FB error'
        print np.median(p_fb_err)
        print np.median(o_fb_err)
        exit()
    ax=sns.boxplot(x="Object", y="p_err", hue="method",hue_order=order,data=df_new, palette=safeuiall,whis=10.0)

    handles, labels = ax.get_legend_handles_labels()
    labels=['Waypoint-interp','Joint-acc','waypoint-fb','Joint-acc-fb']
    #labels=['Open-loop','Fb-ctrl']
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    
    ax.set(ylabel='Position error(cm)')
    plt.show()
    #plt.savefig('/home/bala/plots/'+'fb_pos_err.eps', bbox_inches='tight')
    
    plt.clf()

    ax=sns.boxplot(x="Object", y="o_err", hue="method",hue_order=order,data=df_new, palette=safeuiall,whis=10.0)

    handles, labels = ax.get_legend_handles_labels()
    labels=['Waypoint-interp','Joint-acc','waypoint-fb','Joint-acc-fb']

    #labels=['Open-loop','Fb-ctrl']
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    
    ax.set(ylabel='Orientation error %')
    plt.savefig('/home/bala/plots/'+'fb_orient_err.eps', bbox_inches='tight')
    plt.show()
    #pd_api.write_pd_file('../data/in_grasp_manipulation/'+'w_fb'+'_err.csv',df_new)

def metrics_plots(f_name):
    plt.figure(figsize=(14,6))
    pd_api=pdAPI()
    df=pd_api.read_pd('../data/in_grasp_manipulation/'+f_name)
    err_api=errorAPI()
    err_p=[]
    err_o=[]
    p_time=[]
    meth_arr=[]
    obj_arr=[]

    for i in range(len(df['Object'])):
        txt=df['Desired Pose'][i][1:-1]
        des_pose=np.fromstring(txt,dtype=float,sep=' ')
        txt=df['Reached Pose'][i][1:-1]
        r_pose=np.fromstring(txt,dtype=float,sep=' ')
        p_err,or_err=err_api.pose_err(des_pose,r_pose)
        if(p_err<2.0/100.0):
            err_p.append(p_err*100.0)
            err_o.append(or_err*100.0)
            txt=float(df['Planning Time'][i])
            p_time.append(txt)
            meth_arr.append(df['method'][i])
            obj_arr.append(df['Object'][i])
        
    for i in range(len(df['Object'])):
        txt=df['Desired Pose'][i][1:-1]
        des_pose=np.fromstring(txt,dtype=float,sep=' ')
        txt=df['Reached Pose'][i][1:-1]
        r_pose=np.fromstring(txt,dtype=float,sep=' ')
        p_err,or_err=err_api.pose_err(des_pose,r_pose)
        if(p_err<2.0/100.0):
            err_p.append(p_err*100.0)
            err_o.append(or_err*100.0)
            txt=float(df['Planning Time'][i])
            p_time.append(txt)
            meth_arr.append(df['method'][i])
            obj_arr.append('All Objects')

    # create a new df:
    df_new=pd.DataFrame()
    df_new['Object']=obj_arr
    df_new['p_err']=err_p
    df_new['o_err']=err_o
    df_new['method']=meth_arr
    df_new['p_time']=p_time
    #df_new=df.assign(p_err=err_p,o_err=err_o,p_time=p_time)

    # plot planning time:
    order=['ik_obj','ik_j']
    #order=['ik_obj','fb_ctrl']
    # Compute median errors:
    
    ax=sns.boxplot(x="Object", y="p_time", hue="method",hue_order=order,data=df_new, palette=twoui,whis=10.0)

    handles, labels = ax.get_legend_handles_labels()
    labels=['Waypoint-interp','Joint-acc']
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    

    ax.set(ylabel='Planning Time(s)')
    plt.savefig('/home/bala/plots/'+'p_time.eps', bbox_inches='tight')
    plt.figure(figsize=(14,6))
    ax=sns.boxplot(x="Object", y="p_err", hue="method",hue_order=order,data=df_new, palette=twoui,whis=100.0)

    handles, labels = ax.get_legend_handles_labels()
    labels=['Waypoint-interp','Joint-acc']
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    
    ax.set(ylabel='Position error(cm)')
    plt.savefig('/home/bala/plots/'+'pos_err.eps', bbox_inches='tight')
    plt.figure(figsize=(14,6))
    ax=sns.boxplot(x="Object", y="o_err", hue="method",hue_order=order,data=df_new, palette=twoui)

    handles, labels = ax.get_legend_handles_labels()
    labels=['Waypoint-interp','Joint-acc']
    #labels=['Open-loop','Fb-ctrl']
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    
    ax.set(ylabel='Orientation error %')
    plt.savefig('/home/bala/plots/'+'orient_err.eps', bbox_inches='tight')


def waypoint_plots(f_name_):
    name='p_err.eps'
    cols=['ar_p_err','ar_o_err','db_p_err','db_o_err','ar_db_p','ar_db_o']
    y=cols[0]
    plt.figure(figsize=(14,6))
    # read file:
    pd_api=pdAPI()
    f_name='../data/in_grasp_manipulation/'+f_name_+'_err.csv'
    df=pd_api.read_pd(f_name)


    # Preprocess dataframe:
    columns=['method','Object','P','O']
    df_new=pd_api.create_pd_frame(columns)
    for i in range(len(df['Object'])):
        dat=[]
        dat.append([df['method'][i]+'',df['Object'][i],df['ar_p_err'][i]*100.0,df['ar_o_err'][i]])
        #dat.append([df['method'][i]+'-DB',df['Object'][i],df['db_p_err'][i]*100.0,df['db_o_err'][i]])
        df_new=pd_api.write_pd_data(df_new,dat)
    #ax=sns.boxplot(x="Object", y="P", hue="method",hue_order=['ik_obj-AR','j-AR',
    #                                                          'j-fb-AR'],data=df_new, palette=safeuiall,whis=100)
    ax=sns.boxplot(x="Object", y="P", hue="method",data=df_new, palette=safeuiall,whis=100)

    handles, labels = ax.get_legend_handles_labels()
    
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    
    ax.set(ylabel='Orientation error')
    name='exp_o_err.eps'
    #ax.set(ylabel='Orientation error')
    #ax.set(ylabel='RR cost')
    #plt.ylim(0,0.3)
    #ax.set(ylabel='Position Error(m)')
    plt.savefig('/home/bala/plots/'+name, bbox_inches='tight')
    #plt.show()

def compute_errors(f_name_):
    pd_api=pdAPI()
    f_name='../data/in_grasp_manipulation/'+f_name_+'.csv'
    df=pd_api.read_pd(f_name)
    print df
    err_api=errorAPI()
    dbot_ar_p=[]
    dbot_ar_o=[]
    des_db_p=[]
    des_db_o=[]
    des_ar_p=[]
    des_ar_o=[]
    for i in range(len(df['Object'])):
        txt=df['Desired Pose'][i][1:-1]
        des_pose=np.fromstring(txt,dtype=float,sep=' ')
        txt=df['AR Reached Pose'][i][1:-1]
        ar_pose=np.fromstring(txt,dtype=float,sep=' ')
        txt=df['DBOT Reached Pose'][i][1:-1]
        dbot_pose=np.fromstring(txt,dtype=float,sep=' ')
        p_err,or_err=err_api.pose_err(ar_pose,dbot_pose)
        dbot_ar_p.append(p_err)
        dbot_ar_o.append(or_err)
        p_err,or_err=err_api.pose_err(des_pose,dbot_pose)
        des_db_p.append(p_err)
        des_db_o.append(or_err)
        p_err,or_err=err_api.pose_err(des_pose,ar_pose)
        des_ar_p.append(p_err)
        des_ar_o.append(or_err)

    df_new=df.assign(ar_p_err=des_ar_p,
                     ar_o_err=des_ar_o,
                     db_p_err=des_db_p,
                     db_o_err=des_db_o,
                     ar_db_p=dbot_ar_p,
                     ar_db_o=dbot_ar_o)
    pd_api.write_pd_file('../data/in_grasp_manipulation/'+f_name_+'_err.csv',df_new)
if __name__=='__main__':
    #fb_plots('waypoint_exp_t2.csv')
    #fb_plots('exp_fb.csv')
    #metrics_plots('interpolation_all.csv')
    
    file_name='trials/relaxed-rigidity'
    compute_errors(file_name)
    #waypoint_plots(file_name)
    #'''
