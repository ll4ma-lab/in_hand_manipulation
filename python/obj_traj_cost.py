'''
This cost function tracks a given joint space trajectory and object trajectory on the allegro hand.
ILQR is formulated in the velocity domain. i.e. The input to the robot is a joint velocity and the states are [joint positions, object position]. This allows us to have a continuous, linear dynamics simulation.
To quantify the object dynamics, we assume its rigidly attached to the thumb.
Author: bala@cs.utah.edu
'''
import numpy as np
from numpy.linalg import inv
import PyKDL

def cost(x_des,x,u,dyn_model,t_step=-1):
    # Cost is a quadratic function of maintaining the desired trajectory:
    # x_des=[u_des]
    w_x=dyn_model.w_x
    w_u=dyn_model.w_u
    obj_cost=w_x*np.square(np.linalg.norm(x_des-x))
    input_cost=w_u*np.square(np.linalg.norm(u))
    l=obj_cost+input_cost
    return l

def cost_gradient(x_des,u,dyn_model,t_step=-1):
    w_x=dyn_model.w_x
    w_u=dyn_model.w_u
    obj_cost=w_x*np.square(np.linalg.norm(x_des-x))
    input_cost=w_u*np.square(np.linalg.norm(u))
    l=obj_cost+input_cost
    # Computing analytical gradient for object cost:
    gradient_obj=np.zeros(dyn_model.m)
    gradient_input=np.zeros(dyn_model.m)
    for i in dyn_model.fingers:
        gradient_obj[i*self.DOF:i*self.DOF+self.DOF]=-2.0*(x_des-x)*w_x
        gradient_input[i*self.DOF:i*self.DOF+self.DOF]=-2.0*(u)*w_u
    gradient=gradient_obj+gradient_input
    return gradient

def final_cost(x_des,x,u,dyn_model,t_step=-1):
    # Cost is a quadratic function of maintaining the desired trajectory:
    # x_des=[u_des]
    w_x=dyn_model.w_x
    w_u=dyn_model.w_u
    obj_cost=w_x*np.square(np.linalg.norm(x_des-x))
    input_cost=w_u*np.square(np.linalg.norm(u))
    l=obj_cost+input_cost
    return l

def final_cost_gradient(x_des,u,dyn_model,t_step=-1):
    w_x=dyn_model.w_x
    w_u=dyn_model.w_u
    obj_cost=w_x*np.square(np.linalg.norm(x_des-x))
    input_cost=w_u*np.square(np.linalg.norm(u))
    l=obj_cost+input_cost
    # Computing analytical gradient for object cost:
    gradient_obj=np.zeros(dyn_model.m)
    gradient_input=np.zeros(dyn_model.m)
    for i in dyn_model.fingers:
        gradient_obj[i*self.DOF:i*self.DOF+self.DOF]=-2.0*(x_des-x)*w_x
        gradient_input[i*self.DOF:i*self.DOF+self.DOF]=-2.0*(u)*w_u
    gradient=gradient_obj+gradient_input
    return gradient
