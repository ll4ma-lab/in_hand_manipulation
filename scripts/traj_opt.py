
#import numpy as np
import pyOpt_wrapper_manipulator as pyOpt_wrapper
#import ilqr_with_x as ilqr
#import  manipulator_opt_functions as opt_fn
import  manipulator_opt_functions_time_parsing as opt_fn
class TrajOpt:
    '''
    This class contains trajectory optimization methods, currently has ILQR,
    SNOPT,SLSQP
    Initialization:
    x0= initial state
    x_des= desired state
    u= input to the system(shooting method)
    robot= robot model from robot_models.py
    control='ILQR','SLSQP','SNOPT'
    '''
    def __init__(self,x0,x_des,u,robot,im_cost,fin_cost,grad,grad_fin,control,opt_options={}):
        self.x0=x0
        self.x_des=x_des
        self.u=u
        self.robot=robot
        self.control=control
        self.opt_options=opt_options
        self.opt_fns=opt_fn.OptFns(im_cost,fin_cost,grad,grad_fin,robot,x_des,x0)
        self.im_cost=im_cost
        self.fin_cost=fin_cost
    def optimize(self):
        '''
        if(self.control=='ILQR'):
            controller=ilqr.ILQR(self.robot,self.im_cost,self.fin_cost)
            u_opt,x_opt=controller.controller(self.x0,self.x_des,self.u)
            return u_opt,x_opt
        '''
        if(self.control=='SLSQP'):
            controller=pyOpt_wrapper.OptimalController(self.robot,self.opt_fns,self.control,self.opt_options)
            u_opt,x_opt=controller.learn_controller(self.x0,self.x_des,self.u)
            return u_opt,x_opt

        elif(self.control=='SNOPT'):
            controller=pyOpt_wrapper.OptimalController(self.robot,self.opt_fns,self.control,self.opt_options)
            u_opt,x_opt=controller.learn_controller(self.x0,self.x_des,self.u)
            return u_opt,x_opt
        else:
            controller=pyOpt_wrapper.OptimalController(self.robot,self.opt_fns,self.control,self.opt_options)
            u_opt,x_opt=controller.learn_controller(self.x0,self.x_des,self.u)
            return u_opt,x_opt
            
