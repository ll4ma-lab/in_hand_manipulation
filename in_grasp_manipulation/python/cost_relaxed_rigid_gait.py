# The cost functions are combined into a single function to allow for faster computation
import numpy as np
from numpy.linalg import inv,pinv
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
        T_mat=inv(dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],
                                              f0))*dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
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
        jacobian=pinv(dyn_model.jacobian_full(u[f0*4:f0*4+4],f0))
        gradient_fk[f0*4:f0*4+4]+=-wl*2.0*np.array(np.matrix(diff_pose_d).transpose()*np.matrix(jacobian)).ravel()


    # distance cost
    finger_arr=dyn_model.fingers[dyn_model.fingers!=f0]
    for i in finger_arr:

        f0_mat=dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)
        inv_f0=inv(f0_mat)
        f1_mat=dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
        inv_f1=(f1_mat)
        T_mat=inv_f0*f1_mat
        dist=np.ravel(T_mat[0:3,3])-dyn_model.finger_dist[i]
        const_vec=wr*2.0*dist*dyn_model.obj_dim_weights[0:3]
        #gradient_fk[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF]+=np.ravel(pinv(dyn_model.jacobian_full(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[0:3,:])np.matrix(const_vec)*inv_f0[0:3,0:3])
       
        # for finger f0:
        #T_mat=inv_f1*f0_mat
        #dist=np.ravel(T_mat[0:3,3])+dyn_model.finger_dist[i]
        #const_vec=wr*2.0*dist*dyn_model.obj_dim_weights[0:3]
        #gradient_fk[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF]+=np.ravel(np.matrix(const_vec)*inv_f1[0:3,0:3]*dyn_model.jacobian_full(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[0:3,:])
        
        # orientation cost
        a=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6]
        b=dyn_model.end_effector_pose_array(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6]
        diff=a-b
        diff=np.multiply(diff-dyn_model.orient_rig[i],dyn_model.psi)*dyn_model.obj_dim_weights[3:6]
      
        #gradient_fk[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF]+=2.0*wo*np.ravel(np.matrix(-diff)*dyn_model.jacobian_full(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6,:])
        gradient_fk[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF]+=2.0*wo*pinv(dyn_model.jacobian_full(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6,:])*np.ravel(np.matrix(diff))

    gradient=gradient_fk
    return gradient


def final_cost(x_des,u,dyn_model): 
    wp=dyn_model.wp
    wr=dyn_model.wr
    wo=dyn_model.wo
    l_obj=0.0
    
    # cost for contact points:
    i=3
    finger=3#dyn_model.fingers[i]
    jacobian=dyn_model.jacobian_full(u[finger*4:finger*4+4],finger)[0:3,:]
    gradient_all_joints=np.zeros(4)
    c_obj_pose=dyn_model.object_pose(u,get_T_mat=True)
    f0=i
    f_pose=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)
    diff_pose_d=np.zeros(6)
    off=0.0
    d_pts=[]
    for k in dyn_model.fingers:
        n_pt=np.ravel((c_obj_pose*dyn_model.cpts_mat[k])[0:3,3])
        v_pt,d,_=dyn_model.f_meshes[k].mesh.nearest.on_surface([n_pt])
        d=dyn_model.f_meshes[k].mesh.nearest.signed_distance([n_pt,n_pt])[0]
        #dyn_model.viz.viz_pts([n_pt,v_pt[0]],t='n_cpt')
        if(d>0):
            off+=np.sum(np.abs(n_pt-v_pt[0]))
            d_pts.append(v_pt[0])
    if(len(d_pts)>2):
        l_obj+=off*wp
    
    f0=i
    # distance cost:
    l_f=0.0
    l_orient=0.0
    finger_arr=dyn_model.fingers[dyn_model.fingers!=f0]#np.delete(dyn_model.fingers,3)
    #print dyn_model.finger_dist[0]
    for i in finger_arr:
        T_mat=inv(dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],  f0))*dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
        dist=np.ravel(T_mat[0:3,3])-dyn_model.finger_dist[i]
        l_f+=wr*(np.sum(np.abs(dist)))
        # orientation cost

        a=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6]
        b=dyn_model.end_effector_pose_array(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6]
        diff=a-b
        diff=np.multiply(diff-dyn_model.orient_rig[i],dyn_model.psi)
      
        l_orient+=wo*(np.dot(diff,diff))
    l=l_obj+l_f+l_orient
    print l_obj,l_f,l_orient
    
    return l


def final_cost_gradient(x_des,u,dyn_model):
    wp=dyn_model.wp
    wr=dyn_model.wr
    wo=dyn_model.wo
    gradient_fk=np.zeros(dyn_model.m)
    #x_des in matrix form
    
    # Gradient for f0
    

    finger=3#dyn_model.fingers[i]
    jacobian=pinv(dyn_model.jacobian_full(u[finger*4:finger*4+4],finger))
    gradient_all_joints=np.zeros(4)
    c_obj_pose=dyn_model.object_pose(u,get_T_mat=True)
    obj_pose=dyn_model.object_pose(u)
    f0=finger
    f_pose=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)
    # Using SVD to match the points:
    # Desired points:
    des_pts=[]
    # Nearest points on mesh:
    near_pts=[]

    #diff_pose_d=np.zeros(3)
    off=np.zeros(3)
    for k in dyn_model.fingers:
    #for k in [3]:
        n_pt=np.ravel((c_obj_pose*dyn_model.cpts_mat[k])[0:3,3])
        v_pt,d,_=dyn_model.f_meshes[k].mesh.nearest.on_surface([n_pt])
        d=dyn_model.f_meshes[k].mesh.nearest.signed_distance([n_pt,n_pt])[0]
        if(d>0):
            des_pts.append(n_pt)
            near_pts.append(v_pt[0])
            #off+=np.array(n_pt-v_pt[0])

    if(len(des_pts)>2):
        n_pose=dyn_model.frames.rigid_svd(des_pts,near_pts)
        n_pose=obj_pose+n_pose
        n_f_pose=(dyn_model.frames.T_pose(n_pose)*dyn_model.inv_object_frames[finger])
        n_pose=dyn_model.get_pose_array(n_f_pose)
        diff_pose_d=-1.0*wp*(n_pose-f_pose)*dyn_model.final_obj_dim_weights
        grad=np.array(np.matrix(jacobian)*np.matrix(diff_pose_d).transpose()).ravel()
        gradient_all_joints+=grad
    
    

    gradient_fk[finger*4:finger*4+4]=gradient_all_joints

    f0=finger

    finger_arr=dyn_model.fingers[dyn_model.fingers!=f0]#np.delete(dyn_model.fingers,3)
    f0_mat=dyn_model.end_effector_pose(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)
    inv_f0=inv(f0_mat)

    for i in finger_arr:
        f1_mat=dyn_model.end_effector_pose(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)
        inv_f1=(f1_mat)
        T_mat=inv_f0*f1_mat
        dist=np.ravel(T_mat[0:3,3])-dyn_model.finger_dist[i]
        const_vec=wr*1.0*dist*dyn_model.obj_dim_weights[0:3]
        gradient_fk[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF]+=(np.ravel(pinv(
            dyn_model.jacobian_full(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF], i)[0:3,:])*                                                                         (np.matrix(const_vec)*inv_f0[0:3,0:3]).transpose()))
        # orientation cost
  
        a=dyn_model.end_effector_pose_array(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6]
        b=dyn_model.end_effector_pose_array(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6]
        diff=a-b
        diff=np.multiply(diff-dyn_model.orient_rig[i],dyn_model.psi)*dyn_model.obj_dim_weights[3:6]
      

        #gradient_fk[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF]+=2.0*wo*np.ravel(np.matrix(-diff)*dyn_model.jacobian_full(u[f0*dyn_model.DOF:f0*dyn_model.DOF+dyn_model.DOF],f0)[3:6,:])
        gradient_fk[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF]+=2.0*wo*np.ravel(pinv(dyn_model.jacobian_full(u[i*dyn_model.DOF:i*dyn_model.DOF+dyn_model.DOF],i)[3:6,:])*np.matrix(diff).transpose())
        
    gradient=gradient_fk
    return gradient
