# Computes orientation error between frames
import numpy as np
import csv
import PyKDL
#from numpy.linalg import inv

def file_import(f):
    f_write=open('weights_err.csv','w')
    writer=csv.writer(f_write)
    with open(f,'rb') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='"')
        for row in spamreader:
            line=row
            if line[6] in (None, ""):
                pos_err=-1.0
                R_err=-1.0
            else:
                des_pose=np.array(map(float, line[0:6]))
                reached_pose=np.array(map(float, line[6:12]))
            
                R_des=PyKDL.Rotation.RPY(des_pose[3],des_pose[4],des_pose[5])
                R_reach=PyKDL.Rotation.RPY(reached_pose[3],reached_pose[4],reached_pose[5])

            
                des=np.array(R_des.GetQuaternion())
                reach=np.array(R_reach.GetQuaternion())
                q_sum=np.linalg.norm(des+reach,ord=2)
                q_diff=np.linalg.norm(des-reach,ord=2)
            
                R_err=np.min([q_sum,q_diff])
                #position error:
                pos_err=np.linalg.norm(des_pose[0:3]-reached_pose[0:3],ord=2)
            line.append(pos_err)
            line.append(R_err)
            writer.writerow(line)
            

if __name__=='__main__':
    file_import('errors_weight.csv')
    
