import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class errorAPI:
    def __init(self):
        print 'initialized errorAPI'


    def quat_norm(self,q_des,q_reach):
        q_sum=np.linalg.norm(q_des+q_reach,ord=2)
        q_diff=np.linalg.norm(q_des-q_reach,ord=2)
        R_err=np.min([q_sum,q_diff])/np.sqrt(2.0)
        return R_err

    def pose_err(self,d_pose,r_pose):
        # compute position err:
        position_err=np.linalg.norm(np.ravel(d_pose[0:3]-r_pose[0:3]))
        # orientation error:
        or_err=quat_norm(d_pose[3:],r_pose[3:])
        return [position_err,or_err]
'''
def load_csv(file_name):
    df=pd.read_csv(file_name)
    return df

def read_pose_arr(label):
    des_pose=df_open.as_matrix(columns=[label])
    pose_arr=[]
    for i in range(len(des_pose)):
        des_pose_arr=np.fromstring(des_pose[i][0],dtype=float,sep=' ')
        pose_arr.append(des_pose_arr)
    return pose_arr



if __name__=='__main__':
    df_open=load_csv('../data/in_grasp_manipulation/journal/rr_open_loop.csv')

    d_pose_arr=read_pose_arr(label='desired pose')
    r_pose_arr=read_pose_arr(label='reached pose')
    p_err=[]
    o_err=[]
    for i in range(len(d_pose_arr)):
        err=pose_err(d_pose_arr[i],r_pose_arr[i])
        p_err.append(err[0])
        o_err.append(err[1])

    plt.subplot(2, 1, 1)
    plt.plot(p_err)
    plt.subplot(2,1,2)
    plt.plot(o_err)
    plt.show()
'''
