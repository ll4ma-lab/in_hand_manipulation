#!/usr/bin/env python
# This file uses SNOPT to move the finger to a new location on the object

import rospy
import numpy as np
from grasp_gaits.srv import RelocateFingerKin,RelocateFingerKinResponse
import pyOpt
from geometry_msgs.msg import PointStamped
import trimesh
import copy

# hand model:
import sys
from rospkg import RosPack
rp=RosPack()
rp.list()
path=rp.get_path('ll4ma_kdl')+'/scripts'
sys.path.insert(0,path)

from kdl_allegro_model import allegroKinematics
g_path=rp.get_path('grasp_gaits')
sys.path.insert(0,g_path+'/scripts')

from grasp_class import object as tri_obj


class relocateOpt:
    def __init__(self,des_pt,obj,robot_kdl,finger):
        self.des_pt=des_pt
        self.obj_mesh=obj_mesh
        self.f=finger
        # Robot model:
        self.robot_model=robot_kdl

    def mesh_constraint(self,q):               
        r=0.0
        return r

    def mesh_constraint_grad(self,q):
        grad=np.zeros(16)
        return grad
    
    def stability_constraint(self,q):
        # Add cost for stability:
        # Ensure thumb is forming a equilateral triangle around the other two fingers
        r=0.0
        f_pose=self.robot_model.end_effector_pose_array(q[self.f*4:self.f*4+4],self.f)[0:3]
        f_poses=copy.deepcopy(self.robot_model.f_poses)
        f_poses[self.f]=f_pose
        cent=np.mean(f_poses[0:2],axis=0)
        r=np.sum(np.abs(cent[1]-f_poses[3])[1])
        #print r
        return r
        
            
    def stability_constraint_grad(self,q):
        grad=np.zeros(16)
        
        f_pose=self.robot_model.end_effector_pose_array(q[self.f*4:self.f*4+4],self.f)[0:3]
        f_poses=copy.deepcopy(self.robot_model.f_poses)
        f_poses[self.f]=f_pose
        cent=np.mean(f_poses[0:2],axis=0)
        r=cent[1]-(f_poses[3])[1]
        n_f_pose=(f_pose-self.robot_model.f_poses[self.f])[0:3]
        n_f_pose[0]=0.0
        n_f_pose[2]=0.0
        #print n_f_pose
        grad[self.f*4:self.f*4+4]=1.0*(np.matrix(n_f_pose)
                                        *self.robot_model.jacobian(q[self.f*4:self.f*4+4],self.f))
        return grad
            
    def constraint_fn(self,q):
        G=[]
        G.append(self.stability_constraint(q))
        return G

    def cost_fn(self,q):
        r=np.sum(np.abs(self.des_pt-self.robot_model.end_effector_pose_array(q[self.f*4:self.f*4+4],self.f)[0:3]))
        return r
    
    def cost_fn_gradient(self,q):
        grad=np.zeros(16)
        grad[self.f*4:self.f*4+4]=-1.0*((self.des_pt-
                                    self.robot_model.end_effector_pose_array(q[self.f*4:self.f*4+4],self.f)[0:3])
                                   *self.robot_model.jacobian(q[self.f*4:self.f*4+4],self.f))
        return list(grad)


    def constraint_fn_gradient(self,q):
        G=[]
        grad=self.stability_constraint_grad(q)
        G.append(list(grad))
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
    def __init__(self,obj_mesh,robot_kdl):
        self.obj_trimesh=tri_obj(obj_mesh,False)
        self.robot_kdl=robot_kdl
        #initialize optimization
        print "Initialized relocation server"
                 
    def optimize(self,req):

        #
        t_obj=copy.deepcopy(self.obj_trimesh)
        des_pt=[req.des_contact_pt.point.x,req.des_contact_pt.point.y,req.des_contact_pt.point.z]
        # update obj_pose:
        quat=[req.obj_pose.pose.orientation.w,
              req.obj_pose.pose.orientation.x,
              req.obj_pose.pose.orientation.y,
              req.obj_pose.pose.orientation.z]

        trans=[req.obj_pose.pose.position.x, req.obj_pose.pose.position.y, req.obj_pose.pose.position.z]
        t_obj.transform(quat,trans)
        q0=np.ravel(req.js.position)
        # Get object centroid:
        f_poses=[]
        q=q0
        for i in range(4):
            f_pose=self.robot_kdl.end_effector_pose_array(q[i*4:i*4+4],i)[0:3]
            f_poses.append(f_pose)
        obj_cent=np.mean(f_poses,axis=0)
        print obj_cent
        self.robot_kdl.obj_cent=obj_cent
        self.robot_kdl.f_poses=f_poses
        # 
        wrap=relocateOpt(des_pt,t_obj,self.robot_kdl,req.f_idx)
        
        # setup optimizaiton:
        # Bounds for variables:
        low_bounds=self.robot_kdl.bounds[0]
        up_bounds=self.robot_kdl.bounds[1]

        cnst_bounds_low=np.array([0.0])# plane,ellipse
        cnst_bounds_up=np.array([0.001])# plane,ellipse
        


        opt_prob=pyOpt.Optimization('TP37 Constrained Problem',wrap.objfunc)
        opt_prob.addObj('f')
    
        # Add variables
        opt_prob.addVarGroup('q',len(q0),type='c',value=q0,upper=up_bounds,lower=low_bounds)
        
        # In equality Constraints:
        opt_prob.addConGroup('g',1,type='i',lower=cnst_bounds_low,upper=cnst_bounds_up)

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
        #[fstr,xstr,inform]=optimizer(opt_prob,sens_type='FD')
        [fstr,xstr,inform]=optimizer(opt_prob,sens_type=wrap.grad_obj_func)
        print inform
        print 'Final cost: '+str(fstr)
        q_new=xstr
        n_js=req.js
        n_js.position=q_new
        next_cpt=req.des_contact_pt
        return RelocateFingerKinResponse(next_cpt,n_js,fstr)

if __name__=='__main__':
    # 
    rospy.init_node('relocate_finger_server')
    obj_mesh=g_path+'/data/object_models/banana_bin.stl'
    robot_kdl=allegroKinematics(fingers=[0,1,2,3])
    relocate_server=relocateServer(obj_mesh,robot_kdl)
    
    rospy.Service('regrasp_planner/relocate_finger',RelocateFingerKin ,relocate_server.optimize)
    rospy.spin()
