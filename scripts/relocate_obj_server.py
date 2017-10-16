#!/usr/bin/env python

import rospy
import numpy as np
import pyOpt

from grasp_gaits.msg import Ellipse
from grasp_gaits.srv import RelocateObj, RelocateObjResponse

from geometry_msgs.msg import PoseStamped

class relocateObjOpt:
    def __init__(self,des_pts,ellipses,cpts,obj_pose,f_index):
        self.cpts=cpts
        self.des_pts=des_pts
        self.ellipses=ellipses
        self.obj_pose=obj_pose
        self.f_index=f_index

        self.U_r=[]
        for i in range(len(des_pts)):
            U_r=np.zeros((3,2))
            U_r[0,:]=self.ellipses[i].proj_mat[0:2]
            U_r[1,:]=self.ellipses[i].proj_mat[2:4]
            U_r[2,:]=self.ellipses[i].proj_mat[4:6]
            self.U_r.append(U_r)

        des_pts_2d=[[] for i in range(len(des_pts))]
        cpts_2d=[[] for i in range(len(des_pts))]
        for i in f_index:
            des_pts_2d[i]=np.ravel(np.matrix(des_pts[i]-self.ellipses[i].c)*self.U_r[i])
            cpts_2d[i]=np.ravel(np.matrix(cpts[i]-self.ellipses[i].c)*self.U_r[i])
        self.des_pts_2d=des_pts_2d
        self.cpts_2d=cpts_2d
        
    def plane_constraint(self,q):
        c=[]
        for i in self.f_index:
            # Normals only change when rotated
            a=self.ellipses[i].plane_normal
            d=np.ravel(np.matrix(self.cpts[i]+np.array(q))*np.matrix(a).T)-self.ellipses[i].plane_d
            #d=0.0
            c.append(d)
        return c

    def plane_constraint_grad(self,q):
        grad=np.zeros(3)
        #grad=np.array([self.ellipse.plane_normal[0],self.ellipse.plane_normal[1],self.ellipse.plane_normal[2]])
        return grad

    def ellipse_constraint(self,q):
        c=[]
        for i in self.f_index:
            q_2d=np.ravel(np.matrix(q)*self.U_r[i])
            # Move ellipses
            r=(np.sum(np.abs(self.ellipses[i].f_1-self.cpts_2d[i]-q_2d)))
            +(np.sum(np.abs(q_2d+self.ellipses[i].f_2-self.cpts_2d[i]-q_2d)))#-2*self.ellipses[i].a
            #r=0.0
            c.append(r)
        return c

    def ellipse_constraint_grad(self,q):
        #q_2d=np.ravel(np.matrix(q-self.ellipse.c)*self.U_r)
        grad=np.zeros(3)
        #grad=1.0*np.ravel((np.matrix(self.ellipse.f_1+self.ellipse.f_2)*self.U_r.transpose()))


        return grad
    
    
    def constraint_fn(self,q):
        G=self.plane_constraint(q)+self.ellipse_constraint(q)
        #print G
        return G

    def cost_fn(self,q):
        r=0.0

        for i in self.f_index:
            q_2d=np.ravel(np.matrix(q)*self.U_r[i])
            a=self.ellipses[i].a
            f_1=self.ellipses[i].f_1
            f_2=self.ellipses[i].f_2
            r+=np.amax([2*a,(np.sum(np.abs(f_1-self.des_pts_2d[i]-q_2d))
                                 +np.sum(np.abs(q_2d+f_2-self.des_pts_2d[i]-q_2d)))])-2*a

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



class relocateObjServer:
    def __init__(self):
        print "Initilized object relocation server"
    def optimize(self,req):
        des_pts=[]
        cpts=[]
        for i in range(len(req.des_contact_pts)):
            des_pts.append(np.array([req.des_contact_pts[i].point.x,
                            req.des_contact_pts[i].point.y,
                            req.des_contact_pts[i].point.z]))
            cpts.append(np.array([req.contact_pts[i].point.x,
                         req.contact_pts[i].point.y,
                         req.contact_pts[i].point.z]))

        # setup optimizaiton:
        f_index=[]
        for i in range(len(req.f_index)):
            f_index.append(req.f_index[i].data)
        wrap=relocateObjOpt(des_pts,req.ellipses,cpts,req.obj_pose,f_index)

            
        n_constraints=len(f_index)*2
        # Bounds for variables:
        low_bounds=np.ones(3)*-0.3
        up_bounds=np.ones(3)*0.3
        c_bound_low=np.zeros(n_constraints)
        c_bound_up=np.zeros(n_constraints)
        
        for i in range(n_constraints/2):
            c_bound_up[i*2+1]=2.0*req.ellipses[i].a


        
        
        q0=np.ones(3)*0.0
        
        opt_prob=pyOpt.Optimization('TP37 Constrained Problem',wrap.objfunc)
        opt_prob.addObj('f')
    
        # Add variables
        opt_prob.addVarGroup('q',len(q0),type='c',value=q0,upper=up_bounds,lower=low_bounds)
        
        # In equality Constraints:
        opt_prob.addConGroup('g',n_constraints,type='i',lower=c_bound_low,upper=c_bound_up)

        max_iter=5000
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
        print xstr
        pose=req.obj_pose
        pose.pose.position.x+=xstr[0]
        pose.pose.position.y+=xstr[1]
        pose.pose.position.z+=xstr[2]
        
        return RelocateObjResponse(pose)

if __name__=='__main__':
    rospy.init_node('relocate_obj_server')
    relocate_obj_server=relocateObjServer()
    rospy.Service('/regrasp_planner/relocate_object',RelocateObj, relocate_obj_server.optimize)
    rospy.spin()
