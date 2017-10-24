import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
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
def plot_obj(df): 
    dof=16
    err=[]
    labels=['x','y','z','tx','ty','tz']
    plt.subplot(2, 1, 1)
    for idx in range(3):
        err.append(df.as_matrix(columns=[df.columns[idx+5*dof]]))
        plt.plot(err[idx],label=labels[idx])
    plt.ylabel('Eigen error')
    plt.xlabel('timestep')
    plt.legend()
    plt.subplot(2, 1, 2)
    for idx in range(3,6):
        err.append(df.as_matrix(columns=[df.columns[idx+5*dof]]))
        plt.plot(err[idx],label=labels[idx])
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
    #df_open=load_csv('../data/mpc/mpc_data_open.csv')
    df=load_csv('../data/mpc/mpc_data.csv')

    #idx=int(sys.argv[1])
    #plot_jt_err(int(sys.argv[1]),df)
    #plot_method_err(idx,df_open,df)

    plot_obj(df)
