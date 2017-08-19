#
import rospy
from grasp_gaits.srv import GetEllipse

import numpy as np
import grasp_class
import viz_markers
import tf
from std_msgs.msg import Float64MultiArray

class ellipseApprox:
    def __init__(self,grasp_g):

        self.grasp=grasp_g
        self.grasp.generate_contact_surfaces(True)
        print "Initialized ellipse server"
    
    def service(self,req):
        # transform obj mesh    
        quat=[req.obj_pose.pose.orientation.w,
              req.obj_pose.pose.orientation.x,
              req.obj_pose.pose.orientation.y,
              req.obj_pose.pose.orientation.z]
        trans=[req.obj_pose.pose.position.x, req.obj_pose.pose.position.y, req.obj_pose.pose.position.z]
        self.grasp.object.transform(quat,trans)

        for i in range(len(req.f_index)):
            loc=req.f_index[i].data
            self.grasp.update_contact_surface(loc)
            vert=self.grasp.contact_surfaces[loc].vertices
            vert=self.median_cluster(vert)
            f1,f2,a,U_r=self.fit_ellipse_pca(vert)
            # add ellipse data to result
            print U_r
        res_data=Float64MultiArray()
        self.grasp.reset_object()
        return res_data
    
    def get_ellipse_2d(self,vert_2d):
        #a=max(e_r[:,0])
        #b=max(e_r[:,1])

        f1=[0.1,0.1]
        f2=[0.1,0.1]
        a=0.1
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
        #new_v=(np.matrix(U_r)*e_r.T).T+c
        return f_1,f_2,a,U_r


if __name__=='__main__':
    #
    rospy.init_node('contact_ellipse_server')

    # Get vertices of contact surface:
    robot_meshes=['../data/robot_models/poisson_index.ply',
                  '../data/robot_models/poisson_middle.ply',
                  '../data/robot_models/poisson_ring.ply',
                  '../data/robot_models/poisson_thumb.ply']


    obj_mesh='../data/object_models/box.stl'
    # import object mesh, robot meshes
    grasp_g=grasp_class.grasp([0,1,2,3],robot_meshes,obj_mesh)
    grasp_g.load_object(obj_mesh)


    
    ellipse_server=ellipseApprox(grasp_g)
    rospy.Service('regrasp_planner/update_contact_ellipse',GetEllipse,ellipse_server.service)
    rospy.spin()
