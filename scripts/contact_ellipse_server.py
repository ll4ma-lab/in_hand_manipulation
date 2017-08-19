#!/usr/bin/env python
import rospy
from grasp_gaits.srv import GetEllipse,GetEllipseResponse

import numpy as np
import grasp_class
from viz_markers import VizInHand
import tf
from grasp_gaits.msg import Ellipse
import copy

import sys
from rospkg import RosPack
rp=RosPack()
rp.list()
path_g=rp.get_path('grasp_gaits')

class ellipseApprox:
    def __init__(self,grasp_g):
        self.grasp=grasp_g
        self.grasp.generate_contact_surfaces(True,prefix_path=path_g+'/scripts/')
        print "Initialized ellipse server"
        self.viz=VizInHand(init_node=False)
    def service(self,req):
        # transform obj mesh    
        quat=[req.obj_pose.pose.orientation.w,
              req.obj_pose.pose.orientation.x,
              req.obj_pose.pose.orientation.y,
              req.obj_pose.pose.orientation.z]
        trans=[req.obj_pose.pose.position.x, req.obj_pose.pose.position.y, req.obj_pose.pose.position.z]
        t_obj=copy.deepcopy(self.grasp.object)
        self.grasp.object.transform(quat,trans)

        e_arr=list()
        for i in range(len(req.f_index)):
            loc=req.f_index[i].data
            self.grasp.update_contact_surface(loc)
            vert=self.grasp.contact_surfaces[loc].vertices
            vert=self.median_cluster(vert)
            f1,f2,a,U_r,c,e_r,u_r=self.fit_ellipse_pca(vert)
            # add ellipse data to result
            e_data=Ellipse()
            e_data.a=a
            e_data.f_1=f1
            e_data.f_2=f2
            e_data.proj_mat=U_r
            e_data.c=c
            e_arr.append(e_data)
            if(i==0):
                # viz ellipses
                new_v=(np.matrix(u_r)*e_r.T).T+c
                self.viz.viz_vertices(new_v,t='ellipse')
                self.viz.viz_vertices(vert,t='cs')
        
        self.grasp.object=t_obj

        return GetEllipseResponse(e_arr)
    
    def get_ellipse_2d(self,vert_2d):
        a=max(vert_2d[:,0])
        b=max(vert_2d[:,1])
        c=a**2-b**2
        f1=[c,0.0]
        f2=[-c,0.0]
        return f1,f2,a

    def median_cluster(self,v):
        mean=np.mean(v,axis=0)
        med=np.median(v,axis=0)
        new_v=[]
        for i in range(len(v)):
            if(np.sum(abs(v[i]-mean))>np.sum(abs(v[i]-med))):
                new_v.append(v[i])
        return new_v

    def fit_ellipse_pca(self,v):
        # Find origin and move all vertices to origin
        c=np.median(v,axis=0)
        v=v-c

        # Orient points
        ## Find major and minor axis using pca:
        cov_mat=np.matrix(v.T)*np.matrix(v)
        U,S,V = np.linalg.svd(cov_mat)
        

        U_r=U[:,0:2]
        
        e_r=np.matrix(v)*np.matrix(U_r)

        # Find larges axis in all three dimensions:
        f_1,f_2,a=self.get_ellipse_2d(e_r)
        U_arr=np.zeros(6)
        U_arr[0:2]=U_r[0,:]
        U_arr[2:4]=U_r[1,:]
        U_arr[4:6]=U_r[2,:]
        #new_v=(np.matrix(U_r)*e_r.T).T+c
        return f_1,f_2,a,U_arr,c,e_r,U_r


if __name__=='__main__':
    #
    rospy.init_node('contact_ellipse_server')

    # Get vertices of contact surface:
    robot_meshes=[path_g+'/data/robot_models/poisson_index.ply',
                  path_g+'/data/robot_models/poisson_middle.ply',
                  path_g+'/data/robot_models/poisson_ring.ply',
                  path_g+'/data/robot_models/poisson_thumb.ply']


    obj_mesh=path_g+'/data/object_models/box.stl'
    # import object mesh, robot meshes
    grasp_g=grasp_class.grasp([0,1,2,3],robot_meshes,obj_mesh)
    grasp_g.load_object(obj_mesh)


    
    ellipse_server=ellipseApprox(grasp_g)
    rospy.Service('regrasp_planner/update_contact_ellipse',GetEllipse,ellipse_server.service)
    rospy.spin()
