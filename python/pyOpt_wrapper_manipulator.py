#!/usr/bin/env python

from math import sqrt,fabs
import numpy as np
import pyOpt

class OptimalController:
    def __init__(self,robot,robot_fns,method,opt_options={},DYN_constraint=True):
        self.robot=robot    
        self.robot_fns=robot_fns
        self.method=method
        self.opt_options=opt_options
        self.DYN_constraint=DYN_constraint # This is flag toggles the joint velocity constraint
    def regenerate_control(self,x_n,x0,x_d,u):
        self.x0=x0
        self.x_d=x_d
        low_bounds=[]
        up_bounds=[]
        opt_low_bounds=[]
        opt_up_bounds=[]
        m=self.robot.m
        T=self.robot.T

        # Adding bounds to variables

        low_bounds=self.robot.bounds[0]
        up_bounds=self.robot.bounds[1]

        
        q0=self.robot_fns.get_q(x_n,u)
        for i in xrange(len(q0)/self.robot.m):
           opt_low_bounds.extend(low_bounds)
           opt_up_bounds.extend(up_bounds)

        ## Perform optimization 
        # Create optimization problem
        opt_prob=pyOpt.Optimization('TP37 Constrained Problem',self.objfunc)
        opt_prob.addObj('f')

        if(self.DYN_constraint):
            # Add inequality constraint limits
            lower_limit_velocity=np.ones(T*self.robot.m)*(-0.1)
            upper_limit_velocity=np.ones(T*self.robot.m)*(0.1)

        # Add variables
        opt_prob.addVarGroup('q',len(q0),type='c',value=q0,upper=opt_up_bounds,lower=opt_low_bounds)

        opt_prob.addConGroup('g',self.robot.m*T,type='i',lower=lower_limit_velocity,upper=upper_limit_velocity)
        #opt_prob.addConGroup('g',(T)*self.robot.m+T,type='i',lower=lower_limits,upper=upper_limits)
        method=getattr(pyOpt,self.method)
        optimizer=method(options=self.opt_options)

        [fstr,xstr,inform]=optimizer(opt_prob,sens_type=self.grad_obj_func)
        print inform
        print 'Final cost: '+str(fstr)
        q_star=xstr
        u_star=np.array([[0.0 for i in range(m)]for k in range(T+1)])
        u_star[0,:]=self.x0
        for i in range(T):
           #for k in range(len(self.robot.fingers)):
            u_star[i+1,0:m]=np.array([q_star[i*(m):i*(m)+m]])
        return  u_star
        
    # Creating optimization function to use with pyOpt
    def objfunc(self,q):
        f=self.robot_fns.objectiveFunction(q)
        g=list(self.robot_fns.velocity_constraint(q))#+list(self.robot_fns.rolling_constraint(q))

        fail=0
        return f,g,fail

    def grad_obj_func(self,q,f,g):
        g_obj=np.array([self.robot_fns.objectiveGradient(q)])

        g_con=np.array(list(self.robot_fns.velocity_gradient(q)))#+list(self.robot_fns.rolling_gradient(q)))

        fail=0
        return g_obj,g_con,fail
        
    def learn_controller(self,x0,x_desired,u):
        x=x0
        T=self.robot.T
        t=self.robot.delta_t


        x_n=self.robot.predict_T(x,u)
        x_d=x_desired
        u=self.regenerate_control(x_n,x0,x_d,u)


        x=[]
        return u,x
