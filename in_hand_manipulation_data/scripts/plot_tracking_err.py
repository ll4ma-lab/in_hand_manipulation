import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import PyKDL

def quat_norm(r,p,y):
    R_des=PyKDL.Rotation.RPY(0.0,0.0,0.0)
    des=np.array(R_des.GetQuaternion())
    
    R_cur=PyKDL.Rotation.RPY(r,p,y)
    reach=np.array(R_cur.GetQuaternion())
    
    q_sum=np.linalg.norm(des+reach,ord=2)
    q_diff=np.linalg.norm(des-reach,ord=2)
    R_err=np.min([q_sum,q_diff])/np.sqrt(2.0)

    return R_err
def load_csv(file_name):
    df=pd.read_csv(file_name)
    return df

def plot_method_err(idx,df1,df2):
    dof=16
    err1=df1.as_matrix(columns=[df1.columns[idx]])-df1.as_matrix(columns=[df1.columns[idx+dof]])
    err2=df2.as_matrix(columns=[df2.columns[idx]])-df2.as_matrix(columns=[df2.columns[idx+dof]])
    plt.plot(err1,label='Open-loop error')
    plt.plot(err2,label='MPC error')
    plt.ylabel('Joint angle error(rad)')
    plt.xlabel('timestep')
    plt.legend()
    plt.show()
def plot_obj_method(df_arr, labels):
    dof=16
    plt.subplot(2, 1, 1)
    idx=0
    for i in range(len(df_arr)):
        df1=df_arr[i]
        err_mat=df1.as_matrix(columns=[df1.columns[idx+5*dof],df1.columns[idx+5*dof+1],df1.columns[idx+5*dof+2]])
        err_norm=np.linalg.norm(err_mat,axis=1)
        plt.plot(err_norm,label=labels[i],linewidth=3)
    
    plt.ylabel('2-Norm position error(m)')
    plt.xlabel('timestep')
    plt.legend()

    plt.subplot(2, 1, 2)
    idx=3
    for i in range(len(df_arr)):
        df1=df_arr[i]
        err_mat=df1.as_matrix(columns=[df1.columns[idx+5*dof],df1.columns[idx+5*dof+1],df1.columns[idx+5*dof+2]])
        quat_err=[]
        # compute quaternion error:
        for j in range(len(err_mat[:,0])):
            error=quat_norm(err_mat[j,0],err_mat[j,1],err_mat[j,2])
            quat_err.append(error)
        #err_norm=np.linalg.norm(err_mat,axis=1)
        #plt.plot(err_norm,label=labels[i],linewidth=3)
        plt.plot(quat_err,label=labels[i],linewidth=3)
    plt.ylabel('2-Norm orientation error(rad)')
    plt.xlabel('timestep')
    #plt.legend()
    plt.show()
    
def plot_obj(df): 
    dof=16
    err=[]

    labels=['x','y','z','tx','ty','tz']
    plt.subplot(2, 1, 1)
    for idx in range(3):
        err.append(np.abs(df.as_matrix(columns=[df.columns[idx+5*dof]])))
        plt.plot(err[idx],label=labels[idx])
    idx=0
    err_mat=df.as_matrix(columns=[df.columns[idx+5*dof],df.columns[idx+5*dof+1],df.columns[idx+5*dof+2]])

    err_norm=np.linalg.norm(err_mat,axis=1)
    plt.plot(err_norm,label='2-norm',linewidth=3)

    plt.ylabel('Eigen error')
    plt.xlabel('timestep')
    plt.legend()
    plt.subplot(2, 1, 2)
    for idx in range(3,6):
        err.append(np.abs(df.as_matrix(columns=[df.columns[idx+5*dof]])))
        plt.plot(err[idx],label=labels[idx])
    idx=3
    err_mat=df.as_matrix(columns=[df.columns[idx+5*dof],df.columns[idx+5*dof+1],df.columns[idx+5*dof+2]])

    err_norm=np.linalg.norm(err_mat,axis=1)
    plt.plot(err_norm,label='2-norm',linewidth=3)
    plt.ylabel('Eigen error')
    plt.xlabel('timestep')

    plt.legend()
    plt.show()
    
def plot_jt_err(idx,df):
    dof=16
    err=df.as_matrix(columns=[df.columns[idx]])-df.as_matrix(columns=[df.columns[idx+dof]])

    
    des=df.as_matrix(columns=[df.columns[idx]])
    obs=df.as_matrix(columns=[df.columns[idx+dof]])
    err=des-obs
    j_ff=df.as_matrix(columns=[df.columns[idx+2*dof]])
    j_fb=df.as_matrix(columns=[df.columns[idx+3*dof]])
    
    #plt.plot(des,label='desired')
    #plt.plot(obs,label='observed')

    #plt.plot(des-obs,label='err')
    plt.plot(j_ff,label='ff')
    #plt.plot(j_fb,label='fb')
    #plt.plot(+(dobs)-j_fb,label='control')
    plt.legend()
    plt.show()

if __name__=='__main__':
    df_open=load_csv('../data/mpc/mpc_data_open.csv')
    df_jfb_1=load_csv('../data/mpc/mpc_data_j_fb_0p1.csv')
    df_jfb_5=load_csv('../data/mpc/mpc_data_j_fb_5.csv')
    df_jfb_9=load_csv('../data/mpc/mpc_data_j_fb_9.csv')
    df_ofb=load_csv('../data/mpc/mpc_data_ofb_2.csv')
    df_off_position=load_csv('../data/mpc/mpc_data_off_position.csv')
    df_ofb_position=load_csv('../data/mpc/mpc_data.csv')
    
    #df_arr=[df_open,df_jfb_1,df_jfb_5,df_jfb_9,df_ofb]
    #labels=['open loop','joint fb(0.1)','joint fb(0.5)','joint fb(0.9)','ofb']
    df_arr=[df_open,df_jfb_9,df_ofb,df_ofb_position]
    labels=['open loop','joint error fb','object pose error fb','object position error fb']

    #idx=int(sys.argv[1])
    #plot_jt_err(int(sys.argv[1]),df)
    #plot_method_err(df_open,df)

    #plot_obj(df)
    plot_obj_method(df_arr,labels)
