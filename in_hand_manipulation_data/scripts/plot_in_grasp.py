import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42
safeuiall = ['#ffffff','#fef0d9','#fdcc8a','#fc8d59','#d7301f']
safeui3 = ['#fef0d9','#fdcc8a','#fc8d59','#d7301f']
from data_api import pdAPI
from error_api import errorAPI
import seaborn as sns
import numpy as np
sns.set(font_scale=1.45)
sns.set_style("whitegrid")

def fb_plots(f_name):
    pd_api=pdAPI()
    df=pd_api.read_pd('../data/in_grasp_manipulation/'+f_name)
    err_api=errorAPI()
    err_p=[]
    err_o=[]
    for i in range(len(df['Object'])):
        txt=df['Desired Pose'][i][1:-1]
        des_pose=np.fromstring(txt,dtype=float,sep=' ')
        txt=df['AR Reached Pose'][i][1:-1]
        r_pose=np.fromstring(txt,dtype=float,sep=' ')
        p_err,or_err=err_api.pose_err(des_pose,r_pose)
        err_p.append(p_err)
        err_o.append(or_err)

    df_new=df.assign(p_err=err_p,o_err=err_o)
    #order=['open-ctrl','fb-ctrl']
    order=['ik_obj','fb-ctrl']
    ax=sns.boxplot(x="Object", y="p_err", hue="method",hue_order=order,data=df_new, palette=safeuiall,whis=100)

    handles, labels = ax.get_legend_handles_labels()
    
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    
    ax.set(ylabel='Position error')
    plt.savefig('/home/bala/plots/'+'pos_err.eps', bbox_inches='tight')
    #plt.show()
    ax=sns.boxplot(x="Object", y="o_err", hue="method",hue_order=order,data=df_new, palette=safeuiall,whis=100)

    handles, labels = ax.get_legend_handles_labels()
    
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    
    ax.set(ylabel='Orientation error')
    plt.savefig('/home/bala/plots/'+'orient_err.eps', bbox_inches='tight')
    #plt.show()
    pd_api.write_pd_file('../data/in_grasp_manipulation/'+'exp_fb'+'_err.csv',df_new)
    
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
    #plt.savefig('/home/bala/plots/'+name, bbox_inches='tight')
    plt.show()

def compute_errors(f_name_):
    pd_api=pdAPI()
    f_name='../data/in_grasp_manipulation/'+f_name_+'_exp.csv'
    df=pd_api.read_pd(f_name)
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
    fb_plots('exp_fb.csv')
    '''
    file_name='waypoint'
    compute_errors(file_name)
    waypoint_plots(file_name)
    '''
