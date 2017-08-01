# Here the final cost is the desired fingertip positions and the running cost is the desired fingertip positions at each time step.
import numpy as np
from numpy.linalg import inv
import PyKDL



def cost(x_des,u,dyn_model,t=-1):
    wr=dyn_model.wr
    # cost for object pose:
    l_t=0.0

    des_contacts=dyn_model.time_contacts[int(t)]
    for i in dyn_model.fingers:
        des_pose=np.ravel(des_contacts[i][0:3,3])-np.ravel(dyn_model.end_effector_pose(u[i*4:i*4+4],i)[0:3,3])
        l_t+=np.dot(des_pose,des_pose)

    l_t=wr*l_t
    return l_t


def cost_gradient(x_des,u,dyn_model,t=-1):
    wr=dyn_model.wr

    gradient_fk=np.zeros(dyn_model.m)
    des_contacts=dyn_model.time_contacts[int(t)]

    for i in dyn_model.fingers:
        des_pose=np.ravel(des_contacts[i][0:3,3])-np.ravel(dyn_model.end_effector_pose(u[i*4:i*4+4],i)[0:3,3])
        gradient_fk[i*4:i*4+4]=-wr*des_pose*dyn_model.jacobian(u[i*4:i*4+4],i)

    return gradient_fk

def final_cost(x_des,u,dyn_model): 
    wp=dyn_model.wp
    # cost for object pose:
    l_obj=0.0
    for i in dyn_model.fingers:
        des_pose=np.ravel(dyn_model.des_contact_frames[i][0:3,3])-np.ravel(dyn_model.end_effector_pose(u[i*4:i*4+4],i)[0:3,3])
        l_obj+=np.dot(des_pose,des_pose)
    l=l_obj*wp
    return l


def final_cost_gradient(x_des,u,dyn_model):
    wp=dyn_model.wp
    gradient_fk=np.zeros(dyn_model.m)
    for i in dyn_model.fingers:
        des_pose=np.ravel(dyn_model.des_contact_frames[i][0:3,3])-np.ravel(dyn_model.end_effector_pose(u[i*4:i*4+4],i)[0:3,3])
        gradient_fk[i*4:i*4+4]=-wp*des_pose*dyn_model.jacobian(u[i*4:i*4+4],i)



    gradient=gradient_fk
    return gradient

