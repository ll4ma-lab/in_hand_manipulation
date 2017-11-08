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

def waypoint_plots():
    name='p_err.eps'
    cols=['ar_p_err','ar_o_err','db_p_err','db_o_err','ar_db_p','ar_db_o']
    y=cols[5]
    plt.figure(figsize=(14,6))
    # read file:
    pd_api=pdAPI()
    f_name='../data/in_grasp_manipulation/interpolation_err.csv'
    df=pd_api.read_pd(f_name)

    ax=sns.boxplot(x="Object", y=y, hue="method", data=df, palette=safeui3,whis=100)
    handles, labels = ax.get_legend_handles_labels()
    
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    
    #ax.set(ylabel='Planning Time(s)')
    #ax.set(ylabel='RR cost')
    #plt.ylim(0,0.3)
    #ax.set(ylabel='Position Error(m)')
    #plt.savefig('/home/bala/plots/'+name, bbox_inches='tight')
    plt.show()

def compute_errors():
    pd_api=pdAPI()
    f_name='../data/in_grasp_manipulation/interpolation_exp.csv'
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
    pd_api.write_pd_file('../data/in_grasp_manipulation/interpolation_err.csv',df_new)
def reach_plots():
    name='p_err.eps'
    cols=['Planning Time','RR cost','P_err','O_err']
    y=cols[0]
    plt.figure(figsize=(14,6))
    # read file:
    pd_api=pdAPI()
    f_name='../data/in_grasp_manipulation/interpolation_exp.csv'
    df=pd_api.read_pd(f_name)

    # Compute errors
    err_api=errorAPI()
    '''
    ax=sns.boxplot(x="Object", y=y, hue="method", data=df, palette=safeui3,whis=100)
    handles, labels = ax.get_legend_handles_labels()
    
    ax.legend(handles, labels,title='',loc='upper center', bbox_to_anchor=(0.5, 1.11),ncol=5, fancybox=False, shadow=False)
    
    #ax.set(ylabel='Planning Time(s)')
    #ax.set(ylabel='RR cost')
    #plt.ylim(0,0.3)
    ax.set(ylabel='Position Error(m)')
    #plt.savefig('/home/bala/plots/'+name, bbox_inches='tight')
    plt.show()
    '''
if __name__=='__main__':
    compute_errors()
    waypoint_plots()
