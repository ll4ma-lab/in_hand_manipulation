#import grasp_class
from grasp_gaits.srv import ComputeGDRaw
import numpy as np

from grasp_class import object as tri_obj
import rospy
# hand model:
import sys
from rospkg import RosPack
rp=RosPack()
rp.list()
path=rp.get_path('ll4ma_kdl')+'/scripts'
sys.path.insert(0,path)

from kdl_allegro_model import allegroKinematics
import copy
# service
class gdComputeServer:
    def __init__(self,obj_mesh):
        self.obj_trimesh=tri_obj(obj_mesh,True)
        self.obj_fname=obj_mesh
        # load kdl model:
        self.robot_kdl=allegroKinematics(fingers=[0,1,2,3])
        print self.obj_trimesh.g_mat(0)[1]
        print "initialized gd server"

    def call(self,req):
        t_obj=copy.deepcopy(self.obj_trimesh)
    
        quat=[req.obj_pose[6],
              req.obj_pose[3],
              req.obj_pose[4],
              req.obj_pose[5]]

        trans=[req.obj_pose[0],req.obj_pose[1],req.obj_pose[2]]
        t_obj.transform(quat,trans)
        
        # Get joint angles:
        q=req.js
    
        vert_arr=[]
        pt_arr=[]
        loc=req.f_idx
        # get finger pose
        pt=self.robot_kdl.end_effector_pose_array(q,loc)[0:3]
        pt_arr.append(pt)
        # get closest vertice from object mesh
        eps=0.08

        des_pt=[req.d_cpt[0], req.d_cpt[1], req.d_cpt[2]]
        pt_arr.append(des_pt)
        _,vert_arr=t_obj.mesh.nearest.vertex(pt_arr)
        dist=t_obj.g_mat(vert_arr[1])[vert_arr[0]]
        n_js=np.zeros(4)
        if(req.compute_fd==True):
            #print "computing fd w.r.t. joint angles"
            n_js=np.array(req.js)
            grad=np.zeros(4)
            for i in range(4):
                u=np.zeros(4)
                u[i]=eps
                pt=[]
                pt.append(self.robot_kdl.end_effector_pose_array(n_js+u,loc)[0:3])
                pt.append(self.robot_kdl.end_effector_pose_array(n_js-u,loc)[0:3])
                _,v=t_obj.mesh.nearest.vertex(pt)
                d_p=t_obj.g_mat(vert_arr[1])[v[0]]
                d_m=t_obj.g_mat(vert_arr[1])[v[1]]
                grad[i]=(d_p-d_m)/eps
            n_js=grad
            #print grad
        return dist,n_js

        
if __name__=='__main__':
    # setup ros node
    rospy.init_node('geodesic_server')
    
    # load object mesh and compute geodesic mat:
    obj_name=rospy.get_param('/obj_name')
    
    obj_mesh='/home/bala/object_models/'+obj_name+'/google_16k/nontextured.stl'
    print obj_mesh

    gd_server=gdComputeServer(obj_mesh)

    rospy.Service('in_hand_manipulation/gd_server',ComputeGDRaw,gd_server.call)
    rospy.spin()
    

