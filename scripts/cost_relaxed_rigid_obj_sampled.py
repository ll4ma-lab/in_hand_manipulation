# The cost functions are combined into a single function to allow for faster computation
import numpy as np
from numpy.linalg import inv
import PyKDL

def cost(x_des,u,dyn_model,t_step=-1): 

    wr=dyn_model.wr
    wo=dyn_model.wo
    wl=dyn_model.wl
    f0=3
    l_linear=0.0
    t_step=int(t_step)
    if t_step>0:
        # Adding a cost for linear interpolation of object pose:
        des_finger_pose=np.ravel(dyn_model.linear_finger_poses[t_step])
        diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[f0*4:f0*4+4],f0)
        diff_pose_d=np.unwrap(diff_pose)*dyn_model.obj_dim_weights
        l_linear=wl*np.dot(diff_pose_d,diff_pose_d) 

    # distance cost:
    l_f=0.0
    l_orient=0.0
    finger_arr=dyn_model.fingers[dyn_model.fingers!=f0]
    for i in finger_arr:
        T_mat=inv(dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0))*dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
        dist=np.ravel(T_mat[0:3,3])-dyn_model.finger_dist[i]
        l_f+=wr*(np.dot(dist,dist))
        # orientation cost
        a=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6]
        b=dyn_model.end_effector_pose_array(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6]
        diff=a-b
        diff[2]=0.0
        diff[0]=0.0
        diff[1]=diff[1]-dyn_model.yaw_rig[f0,i]
        l_orient+=wo*(np.dot(diff,diff))
    
    l=l_f+l_orient+l_linear
    
    return l
def cost_gradient(x_des,u,dyn_model,t_step=-1):
    wp=dyn_model.wp
    wr=dyn_model.wr
    wo=dyn_model.wo    
    wl=dyn_model.wl
    gradient_fk=np.zeros(dyn_model.m)
    t_step=int(t_step)
    f0=3

    if t_step>0:
        # Adding a cost for linear interpolation of object pose:
        des_finger_pose=np.ravel(dyn_model.linear_finger_poses[t_step])
        diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[f0*4:f0*4+4],f0)
        diff_pose_d=np.unwrap(diff_pose)*dyn_model.obj_dim_weights
        jacobian=dyn_model.jacobian_full(u[f0*4:f0*4+4],f0)
        gradient_fk[f0*4:f0*4+4]+=-wl*2.0*np.array(np.matrix(diff_pose_d)*np.matrix(jacobian)).ravel()


    # distance cost
    finger_arr=dyn_model.fingers[dyn_model.fingers!=f0]
    for i in finger_arr:

        f0_mat=dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)
        inv_f0=inv(f0_mat)
        f1_mat=dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
        inv_f1=inv(f1_mat)
        T_mat=inv_f0*f1_mat
        dist=np.ravel(T_mat[0:3,3])-dyn_model.finger_dist[i]
        const_vec=wr*2.0*dist*dyn_model.obj_dim_weights[0:3]
        gradient_fk[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF]+=np.ravel(np.matrix(const_vec)*inv_f0[0:3,0:3]*dyn_model.jacobian_full(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[0:3,:])
       
        # for finger f0:
        T_mat=inv_f1*f0_mat
        dist=np.ravel(T_mat[0:3,3])+dyn_model.finger_dist[i]
        const_vec=wr*2.0*dist*dyn_model.obj_dim_weights[0:3]
        gradient_fk[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF]+=np.ravel(np.matrix(const_vec)*inv_f1[0:3,0:3]*dyn_model.jacobian_full(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[0:3,:])
        
        # orientation cost
        a=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6]
        b=dyn_model.end_effector_pose_array(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6]
        diff=a-b
        diff=np.multiply(diff-dyn_model.orient_rig[i],dyn_model.psi)*dyn_model.obj_dim_weights[3:6]
      
        gradient_fk[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF]+=2.0*wo*np.ravel(np.matrix(-diff)*dyn_model.jacobian_full(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6,:])
        gradient_fk[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF]+=2.0*wo*np.ravel(np.matrix(diff)*dyn_model.jacobian_full(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6,:])

    gradient=gradient_fk
    return gradient


def final_cost(x_des,u,dyn_model): 
    wp=dyn_model.wp
    wr=dyn_model.wr
    wo=dyn_model.wo
    l_obj=0.0
    # cost for object pose:
    i=3
    des_finger_pose=dyn_model.des_finger_pose
    diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[i*4:i*4+4],i)
    diff_pose_d=np.unwrap(diff_pose)*dyn_model.final_obj_dim_weights
    l_obj=wp*np.dot(diff_pose_d,diff_pose_d) 
    #print des_finger_pose
    f0=i
    # distance cost:
    l_f=0.0
    l_orient=0.0
    finger_arr=dyn_model.fingers[dyn_model.fingers!=f0]#np.delete(dyn_model.fingers,3)
    for i in finger_arr:
        T_mat=inv(dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0))*dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
        dist=np.ravel(T_mat[0:3,3])-dyn_model.finger_dist[i]
        l_f+=wr*(np.dot(dist,dist))

        # orientation cost


        a=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6]
        b=dyn_model.end_effector_pose_array(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6]
        diff=a-b
        diff=np.multiply(diff-dyn_model.orient_rig[i],dyn_model.psi)
      
        l_orient+=wo*(np.dot(diff,diff))
    l=l_obj+l_f+l_orient
    
    return l


def final_cost_gradient(x_des,u,dyn_model):
    wp=dyn_model.wp
    wr=dyn_model.wr
    wo=dyn_model.wo
    gradient_fk=np.zeros(dyn_model.m)
    #x_des in matrix form
    
    # Gradient for f0
    
    i=3
    finger=3#dyn_model.fingers[i]
    jacobian=dyn_model.jacobian_full(u[finger*4:finger*4+4],finger)
    

    des_finger_pose=dyn_model.des_finger_pose
    diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[finger*4:finger*4+4],finger)

    diff_pose_d=np.unwrap(diff_pose)*dyn_model.final_obj_dim_weights

    gradient_all_joints=np.array(np.matrix(diff_pose_d)*np.matrix(jacobian)).ravel()
        
    gradient_fk[finger*4:finger*4+4]=-wp*2.0*gradient_all_joints

    f0=i

    finger_arr=dyn_model.fingers[dyn_model.fingers!=f0]#np.delete(dyn_model.fingers,3)
    for i in finger_arr:
        f0_mat=dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)
        inv_f0=inv(f0_mat)
        f1_mat=dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
        inv_f1=inv(f1_mat)
        T_mat=inv_f0*f1_mat
        dist=np.ravel(T_mat[0:3,3])-dyn_model.finger_dist[i]
        const_vec=wr*2.0*dist*dyn_model.obj_dim_weights[0:3]
        gradient_fk[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF]+=np.ravel(np.matrix(const_vec)*inv_f0[0:3,0:3]*dyn_model.jacobian_full(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[0:3,:])
       
        # for finger 0:
        T_mat=inv_f1*f0_mat
        dist=np.ravel(T_mat[0:3,3])+dyn_model.finger_dist[i]
        const_vec=wr*2.0*dist*dyn_model.obj_dim_weights[0:3]
        gradient_fk[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF]+=np.ravel(np.matrix(const_vec)*inv_f1[0:3,0:3]*dyn_model.jacobian_full(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[0:3,:])
       
        # orientation cost
  
        a=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6]
        b=dyn_model.end_effector_pose_array(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6]
        diff=a-b
        diff=np.multiply(diff-dyn_model.orient_rig[i],dyn_model.psi)*dyn_model.obj_dim_weights[3:6]
      

        gradient_fk[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF]+=2.0*wo*np.ravel(np.matrix(-diff)*dyn_model.jacobian_full(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6,:])
        gradient_fk[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF]+=2.0*wo*np.ravel(np.matrix(diff)*dyn_model.jacobian_full(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6,:])
        
    gradient=gradient_fk
    return gradient

def cost_terms(u_joints,dyn_model): 
    f0=3
    l_linear=0.0
    l_f=0.0
    l_orient=0.0

    for t_step_ in range(dyn_model.T-1):
        t_step=int(t_step_)
        u=u_joints[t_step]
        if t_step>0:
            # Adding a cost for linear interpolation of object pose:
            des_finger_pose=np.ravel(dyn_model.linear_finger_poses[t_step])
            diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[f0*4:f0*4+4],f0)
            diff_pose_d=np.unwrap(diff_pose)*dyn_model.obj_dim_weights
            l_linear+=np.dot(diff_pose_d,diff_pose_d) 
        # distance cost:  
        finger_arr=dyn_model.fingers[dyn_model.fingers!=f0]
        for i in finger_arr:
            T_mat=inv(dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0))*dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
            dist=np.ravel(T_mat[0:3,3])-dyn_model.finger_dist[i]
            l_f+=(np.dot(dist,dist))
            # orientation cost
            a=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6]
            b=dyn_model.end_effector_pose_array(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6]
            diff=a-b
            diff[2]=0.0
            diff[0]=0.0
            diff[1]=diff[1]-dyn_model.yaw_rig[f0,i]
            l_orient+=(np.dot(diff,diff))
    u=u_joints[-1]
    #final cost
    i=3
    des_finger_pose=dyn_model.des_finger_pose
    diff_pose=des_finger_pose-dyn_model.end_effector_pose_array(u[i*4:i*4+4],i)
    diff_pose_d=np.unwrap(diff_pose)*dyn_model.obj_dim_weights
    l_obj=np.dot(diff_pose_d,diff_pose_d) 
    #print des_finger_pose
    f0=i
    # distance cost:

    finger_arr=dyn_model.fingers[dyn_model.fingers!=f0]#np.delete(dyn_model.fingers,3)
    for i in finger_arr:
        T_mat=inv(dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0))*dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
        dist=np.ravel(T_mat[0:3,3])-dyn_model.finger_dist[i]
        l_f+=(np.dot(dist,dist))

        # orientation cost


        a=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6]
        b=dyn_model.end_effector_pose_array(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6]
        diff=a-b
        diff=np.multiply(diff-dyn_model.orient_rig[i],dyn_model.psi)
      
        l_orient+=(np.dot(diff,diff))



    return l_f,l_orient,l_linear,l_obj

