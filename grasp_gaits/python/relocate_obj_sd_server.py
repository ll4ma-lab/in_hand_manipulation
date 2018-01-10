#!/usr/bin/env python
import rospy
import numpy as np
import pyOpt
import PyKDL
import frames
from grasp_gaits.msg import Ellipse
from grasp_gaits.srv import RelocateObjSd, RelocateObjSdResponse

from geometry_msgs.msg import PoseStamped
import sys
from rospkg import RosPack
rp=RosPack()
rp.list()
path_g=rp.get_path('grasp_gaits')
import grasp_class
class relocateObjOpt:
    def __init__(self,des_pts,f_index,f_mesh,c_pts,q0):
        self.f_mesh=f_mesh
        self.des_pts=des_pts
        self.c_pts=c_pts
        self.f_index=f_index
        self.frames=frames.transformMat()
        # build T mats for obj_pose to des_pts:
        des_pts_mat=[]
        p_T_o=self.frames.T_pose(q0)
        o_T_p=np.matrix(np.linalg.inv(p_T_o))
        for i in range(len(des_pts)):
            p_T_c=np.eye(4)
            p_T_c[0:3,3]=des_pts[i]
            o_T_c=o_T_p*p_T_c
            des_pts_mat.append(o_T_c)
        self.des_pts_mat=des_pts_mat

    def cost_fn(self,q):
        r=0.0
        # Transform points:
        T=self.frames.T_pose(q)
        for i in self.f_index:
            pt=np.ravel((T*self.des_pts_mat[i])[0:3,3])
            d=self.f_mesh[i].mesh.nearest.signed_distance([pt,pt])[0]
            if(d>0):
                v_pt,d,_=self.f_mesh[i].mesh.nearest.on_surface([pt])
                r+=np.sum(np.abs(pt-v_pt[0]))
        print r
        return r
    
    def cost_fn_gradient(self,q):
        grad=np.zeros(6)
        T=self.frames.T_pose(q)
        # Using SVD to match the points:
        # Desired points:
        des_pts=[]
        # Nearest points on mesh:
        near_pts=[]
        t_d=0.0
        for i in self.f_index:
            pt=np.ravel((T*self.des_pts_mat[i])[0:3,3])

            d=self.f_mesh[i].mesh.nearest.signed_distance([pt,pt])[0]
            if(d>0):
                t_d+=np.abs(d)
                v_pt,d,_=self.f_mesh[i].mesh.nearest.on_surface([pt])
                des_pts.append(pt)
                near_pts.append(v_pt[0])
       
        if(len(des_pts)>2):
            # Finding rigid transformation to reduce distance between des_pts and near_pts
            n_pose=self.frames.rigid_svd(des_pts,near_pts)
            grad=-1.0*n_pose
        #elif(t_d>0):
        #    for i in range(len(des_pts)):
        #        n_pose=des_pts[i]-near_pts[i]
        #        grad[0:3]+=1.0*n_pose
        #print grad
        return list(grad)
    
    def objfunc(self,q):
        f=self.cost_fn(q)
        g=[]#list(self.constraint_fn(q))
        fail=0
        return f,g,fail

    def grad_obj_func(self,q,f,g):
        g_obj=np.array([self.cost_fn_gradient(q)])
        g_con=[]#=self.constraint_fn_gradient(q)
        fail=0
        return g_obj,g_con,fail



class relocateObjServer:
    def __init__(self,f_meshes):
        print "Initilized object relocation server"
        self.f_meshes=f_meshes
        self.frames=frames.transformMat()
    def optimize(self,req):
        des_pts=[]
        c_pts=[]
        for i in range(len(req.des_contact_pts)):
            des_pts.append(np.array([req.des_contact_pts[i].point.x,
                                     req.des_contact_pts[i].point.y,
                                     req.des_contact_pts[i].point.z]))
            c_pts.append(np.array([req.contact_pts[i].point.x,
                                     req.contact_pts[i].point.y,
                                     req.contact_pts[i].point.z]))
         
        # setup optimizaiton:
        f_index=[]
        for i in range(len(req.f_index)):
            f_index.append(req.f_index[i].data)

            
        n_constraints=0
        # Bounds for variables:
        low_bounds=np.ones(6)*-3.14
        low_bounds[0:3]=-2.5
        up_bounds=np.ones(6)*3.14
        up_bounds[0:3]=2.5
        c_bound_low=np.zeros(n_constraints)
        c_bound_up=np.zeros(n_constraints)

        # Get current object pose:
        q0=np.ones(6)*0.0
        q0[0]=req.obj_pose.pose.position.x
        q0[1]=req.obj_pose.pose.position.y
        q0[2]=req.obj_pose.pose.position.z
        R=PyKDL.Rotation()
        R=R.Quaternion(req.obj_pose.pose.orientation.x, req.obj_pose.pose.orientation.y,  req.obj_pose.pose.orientation.z, req.obj_pose.pose.orientation.w)

        q0[3], q0[4], q0[5]=R.GetRPY()

        wrap=relocateObjOpt(des_pts,f_index,self.f_meshes,c_pts,q0)
                
        opt_prob=pyOpt.Optimization('TP37 Constrained Problem',wrap.objfunc)
        opt_prob.addObj('f')
    
        # Add variables
        opt_prob.addVarGroup('q',len(q0),type='c',value=q0,upper=up_bounds,lower=low_bounds)
        
        # In equality Constraints:
        #opt_prob.addConGroup('g',n_constraints,type='i',lower=c_bound_low,upper=c_bound_up)

        max_iter=50
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
        print xstr
        pose=req.obj_pose
        pose.pose.position.x=xstr[0]
        pose.pose.position.y=xstr[1]
        pose.pose.position.z=xstr[2]
        quat=self.frames.quat_RPY(xstr[3:6])
        pose.pose.orientation.x=quat[0]
        pose.pose.orientation.y=quat[1]
        pose.pose.orientation.z=quat[2]
        pose.pose.orientation.w=quat[3]
        
        return RelocateObjSdResponse(pose)

if __name__=='__main__':
    rospy.init_node('relocate_obj_server')
    # load finger mesh
    # Get vertices of contact surface:
    robot_meshes=[path_g+'/data/robot_models/poisson_index.ply',
                  path_g+'/data/robot_models/poisson_middle.ply',
                  path_g+'/data/robot_models/poisson_ring.ply',
                  path_g+'/data/robot_models/cvx_thumb.ply']

    obj_mesh=path_g+'/data/object_models/box.stl'
    # import object mesh, robot meshes
    grasp_g=grasp_class.grasp([0,1,2,3],robot_meshes,obj_mesh)
    
    relocate_obj_server=relocateObjServer(grasp_g.finger_meshes)
    rospy.Service('/regrasp_planner/relocate_object',RelocateObjSd, relocate_obj_server.optimize)
    rospy.spin()
