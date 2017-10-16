# The cost functions are combined into a single function to allow for faster computation
import numpy as np
from numpy.linalg import inv
import PyKDL

def cost(x_des,u,dyn_model,t=-1): 
    wr=dyn_model.wr
    l_obj=0.0
    f=3
    cur_pose=dyn_model.object_pose(u,f)

    obj_mat=np.eye(4)

    R=PyKDL.Rotation.RPY(cur_pose[3],cur_pose[4],cur_pose[5])
    for k in range(3):
        for i in range(3):
            obj_mat[k,i]=R[k,i]
    obj_mat[0:3,3]=cur_pose[0:3]

    for i in dyn_model.fingers:
        des_finger_pose=dyn_model.object_finger_diff(obj_mat,i)   
        diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[i*4:i*4+4],i)
        diff_pose_d=np.unwrap(diff_pose)*dyn_model.obj_dim_weights

        l_obj+=wr*np.dot(diff_pose_d,diff_pose_d) 

    l=l_obj

    return l_obj


def cost_gradient(x_des,u,dyn_model,t=-1):

    wr=dyn_model.wr

    gradient_fk=np.zeros(dyn_model.m)
    f=3
    cur_pose=dyn_model.object_pose(u,f)

    obj_mat=np.eye(4)

    R=PyKDL.Rotation.RPY(cur_pose[3],cur_pose[4],cur_pose[5])
    for k in range(3):
        for i in range(3):
            obj_mat[k,i]=R[k,i]
    obj_mat[0:3,3]=cur_pose[0:3]

    # Gradient for f0
    for i in dyn_model.fingers:
        finger=i#dyn_model.fingers[i]
        jacobian=dyn_model.jacobian_full(u[finger*4:finger*4+4],finger)
        des_finger_pose=dyn_model.object_finger_diff(obj_mat,finger)
        diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[finger*4:finger*4+4],finger)

        diff_pose_d=np.unwrap(diff_pose)*dyn_model.obj_dim_weights

        gradient_all_joints=np.array(np.matrix(diff_pose_d)*np.matrix(jacobian)).ravel()
        
        gradient_fk[finger*4:finger*4+4]+=-wr*2.0*gradient_all_joints

    gradient=gradient_fk

    return gradient_fk


def final_cost(x_des,u,dyn_model): 
    wp=dyn_model.wp
    obj_mat=np.eye(4)
    R=PyKDL.Rotation.RPY(x_des[3],x_des[4],x_des[5])
    for k in range(3):
        for i in range(3):
            obj_mat[k,i]=R[k,i]
    obj_mat[0:3,3]=x_des[0:3]
    l_obj=0.0
    # cost for object pose:
    
    for i in dyn_model.fingers:
        des_finger_pose=dyn_model.object_finger_diff(obj_mat,i)   
        diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[i*4:i*4+4],i)
        diff_pose_d=np.unwrap(diff_pose)*dyn_model.obj_dim_weights
        l_obj+=wp*np.dot(diff_pose_d,diff_pose_d) 

    l=l_obj
    
    return l



def final_cost_gradient(x_des,u,dyn_model):
    wp=dyn_model.wp
    gradient_fk=np.zeros(dyn_model.m)
    #x_des in matrix form
    obj_mat=np.ones([4,4])
    R=PyKDL.Rotation.RPY(x_des[3],x_des[4],x_des[5])
    for k in range(3):
        for i in range(3):
            obj_mat[k,i]=R[k,i]
    obj_mat[0:3,3]=x_des[0:3]
    
    # Gradient for f0
    for i in dyn_model.fingers:
        finger=i#dyn_model.fingers[i]
        jacobian=dyn_model.jacobian_full(u[finger*4:finger*4+4],finger)
        des_finger_pose=dyn_model.object_finger_diff(obj_mat,finger)
        diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[finger*4:finger*4+4],finger)

        diff_pose_d=np.unwrap(diff_pose)*dyn_model.obj_dim_weights

        gradient_all_joints=np.array(np.matrix(diff_pose_d)*np.matrix(jacobian)).ravel()
        
        gradient_fk[finger*4:finger*4+4]+=-wp*2.0*gradient_all_joints
       
    gradient=gradient_fk
    return gradient

