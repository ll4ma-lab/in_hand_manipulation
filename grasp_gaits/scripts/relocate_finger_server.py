#!/usr/bin/env python
# This file uses SNOPT to move the finger to a new location on the object

import rospy
import numpy as np
from grasp_gaits.msg import Ellipse
from grasp_gaits.srv import RelocateFinger,RelocateFingerResponse
import pyOpt
from geometry_msgs.msg import PointStamped


class relocateOpt:
    def __init__(self,des_pt,ellipse):
        self.des_pt=des_pt
        self.ellipse=ellipse
        U_r=np.zeros((3,2))
        U_r[0,:]=self.ellipse.proj_mat[0:2]
        U_r[1,:]=self.ellipse.proj_mat[2:4]
        U_r[2,:]=self.ellipse.proj_mat[4:6]
        self.ellipse.f_1=np.ravel(self.ellipse.f_1)
        self.ellipse.f_2=np.ravel(self.ellipse.f_2)
        self.ellipse.c=np.ravel(self.ellipse.c)
        # Turn it to a matrix:
        self.U_r=U_r

    def plane_constraint(self,q):
        a=self.ellipse.plane_normal
        d=np.ravel(np.matrix(q)*np.matrix(a).T)-self.ellipse.plane_d
        return d

    def plane_constraint_grad(self,q):
        grad=np.array([self.ellipse.plane_normal[0],self.ellipse.plane_normal[1],self.ellipse.plane_normal[2]])
        return grad

    def ellipse_constraint(self,q):        
        q_2d=np.ravel(np.matrix(q-self.ellipse.c)*self.U_r)
        r=(np.sum(np.abs(self.ellipse.f_1-q_2d)))+(np.sum(np.abs(self.ellipse.f_2-q_2d)))-2*self.ellipse.a
        return r

    def ellipse_constraint_grad(self,q):
        #q_2d=np.ravel(np.matrix(q-self.ellipse.c)*self.U_r)
        grad=np.zeros(3)
        grad=1.0*np.ravel((np.matrix(self.ellipse.f_1+self.ellipse.f_2)*self.U_r.transpose()))


        return grad
    
    
    def constraint_fn(self,q):
        G=[]
        G.append(self.plane_constraint(q))
        G.append(self.ellipse_constraint(q))
        return G

    def cost_fn(self,q):
        r=np.sum(np.abs(self.des_pt[0:3]-q[0:3]))
        return r
    
    def cost_fn_gradient(self,q):
        grad=np.zeros(3)
        grad[0:3]=-1.0*(self.des_pt[0:3]-q[0:3])
        return list(grad)


    def constraint_fn_gradient(self,q):
        G=[]
        grad1=self.plane_constraint_grad(q)
        G.append(list(grad1))
        grad2=self.ellipse_constraint_grad(q)
        G.append(list(grad2))
        return G
    
    def objfunc(self,q):
        f=self.cost_fn(q)
        g=list(self.constraint_fn(q))
        fail=0
        return f,g,fail

    def grad_obj_func(self,q,f,g):
        g_obj=np.array([self.cost_fn_gradient(q)])
        g_con=self.constraint_fn_gradient(q)
        fail=0
        return g_obj,g_con,fail


class relocateServer:
    def __init__(self):
        #initialize optimization
        print "Initialized relocation server"
                 
    def optimize(self,req):
        #
        des_pt=[req.des_contact_pt.point.x,req.des_contact_pt.point.y,req.des_contact_pt.point.z]
        wrap=relocateOpt(des_pt,req.ellipse)
        # setup optimizaiton:
        # Bounds for variables:
        low_bounds=np.ones(3)*-0.5
        up_bounds=np.ones(3)*0.5
        
        cnst_bounds_low=np.array([0.0,-req.ellipse.a*2.0])# plane,ellipse
        cnst_bounds_up=np.array([0.0,0.0])# plane,ellipse
        
        q0=np.ones(3)*0.1
        
        opt_prob=pyOpt.Optimization('TP37 Constrained Problem',wrap.objfunc)
        opt_prob.addObj('f')
    
        # Add variables
        opt_prob.addVarGroup('q',len(q0),type='c',value=q0,upper=up_bounds,lower=low_bounds)
        
        # In equality Constraints:
        opt_prob.addConGroup('g',2,type='i',lower=cnst_bounds_low,upper=cnst_bounds_up)

        max_iter=1000
        '''
        method=getattr(pyOpt,'SLSQP')
        slsqp_options={'MAXIT':max_iter,'ACC':1e-4}
        optimizer=method(options=slsqp_options)
        '''
        
        method=getattr(pyOpt,'SNOPT')
        #opt_options={'Iterations limit':max_iter,'Solution':'No','iPrint':0,'iSumm':0,'Problem Type':'Feasible point'}
        opt_options={'Iterations limit':max_iter,'Solution':'Yes','iPrint':1,'iSumm':1,'Problem Type':'Minimize'}
        optimizer=method(options=opt_options)
        print "Optimizing"
        [fstr,xstr,inform]=optimizer(opt_prob,sens_type='FD')
        #[fstr,xstr,inform]=optimizer(opt_prob,sens_type=wrap.grad_obj_func)
        print inform
        print 'Final cost: '+str(fstr)

        next_cpt=PointStamped()
        next_cpt.point.x=xstr[0]
        next_cpt.point.y=xstr[1]
        next_cpt.point.z=xstr[2]
        return RelocateFingerResponse(next_cpt,fstr)

if __name__=='__main__':
    # 
    rospy.init_node('relocate_finger_server')
    relocate_server=relocateServer()
    rospy.Service('regrasp_planner/relocate_finger',RelocateFinger ,relocate_server.optimize)
    rospy.spin()
