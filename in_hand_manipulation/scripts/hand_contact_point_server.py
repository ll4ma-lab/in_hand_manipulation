#!/usr/bin/env python
import rospy
from in_hand_manipulation.srv import GetHandContact
import numpy as np
import trimesh
import copy
from geometry_msgs.msg import PointStamped

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

class handContactServer:
    def __init__(self,obj_mesh):
        # load object mesh:
        self.obj_trimesh=tri_obj(obj_mesh,False)
        self.obj_fname=obj_mesh
        # load kdl model:
        self.robot_kdl=allegroKinematics(fingers=[0,1,2,3])
        
        print 'Initialized hand contact server'

    def update_mesh(self,obj_mesh):
        self.obj_trimesh=tri_obj(obj_mesh,False)
        self.obj_fname=obj_mesh

    def call(self,req):
        obj_name=rospy.get_param('/obj_name')
        
        obj_mesh='/home/bala/object_models/'+obj_name+'/google_16k/nontextured.stl'
        self.update_mesh(obj_mesh)
        # move object mesh:
        t_obj=copy.deepcopy(self.obj_trimesh)
        if(req.transform_points==True):
            i_obj=copy.deepcopy(self.obj_trimesh)
            quat=[req.initial_obj_pose.pose.orientation.w,
                  req.initial_obj_pose.pose.orientation.x,
                  req.initial_obj_pose.pose.orientation.y,
                  req.initial_obj_pose.pose.orientation.z]

            trans=[req.initial_obj_pose.pose.position.x, req.initial_obj_pose.pose.position.y, req.initial_obj_pose.pose.position.z]
            i_obj.transform(quat,trans)

        else:
            i_obj=t_obj
            
        quat=[req.obj_pose.pose.orientation.w,
              req.obj_pose.pose.orientation.x,
              req.obj_pose.pose.orientation.y,
              req.obj_pose.pose.orientation.z]

        trans=[req.obj_pose.pose.position.x, req.obj_pose.pose.position.y, req.obj_pose.pose.position.z]
        t_obj.transform(quat,trans)
        
        # Get joint angles:
        q=req.joint_state.position
    
        vert_arr=[]
        pt_arr=[]
        for i in range(len(req.f_index)):
            loc=req.f_index[i].data
            # get finger pose
            pt=self.robot_kdl.end_effector_pose_array(q[loc*4:loc*4+4],loc)[0:3]
            pt_arr.append(pt)
            # get closest vertice from object mesh
        _,vert_arr=t_obj.mesh.nearest.vertex(pt_arr)

        # Get contact point values on mesh:
        c_pts=[]
        for i in vert_arr:
            cpt=PointStamped()
            pt=i_obj.mesh.vertices[i]
            cpt.point.x=pt[0]
            cpt.point.y=pt[1]
            cpt.point.z=pt[2]
            c_pts.append(cpt)

        
        return vert_arr,c_pts
    
if __name__=='__main__':
    #
    rospy.init_node('hand_contact_server')

    obj_name=rospy.get_param('/obj_name')
    
    obj_mesh='/home/bala/object_models/'+obj_name+'/google_16k/nontextured.stl'
    print obj_mesh
    hand_server=handContactServer(obj_mesh)

    rospy.Service('in_hand_manipulation/get_hand_contacts',GetHandContact,hand_server.call)
    rospy.spin()
