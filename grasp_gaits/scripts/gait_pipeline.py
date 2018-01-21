#import rospy
from gait_client import *
import sys
from rospkg import RosPack
rp=RosPack()
path_=rp.get_path('optimization_pkg')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *

path_=rp.get_path('grasp_gaits')+'/python'
sys.path.insert(0,path_)
from viz_markers import VizInHand
from geometry_msgs.msg import PoseStamped

import pickle
import PyKDL
from numpy.linalg import inv
import copy
import frames
path_=rp.get_path('in_grasp_manipulation')+'/scripts'
sys.path.insert(0,path_)
from in_grasp_client import inGraspClient
class graspGait:
    def __init__(self,o_mesh,weights):
        self.fg_client=fgClient()
        self.ig_client=inGraspClient(weights)
        self.traj_server=trajectoryServer(10,init_node=True,topic_name='/in_grasp/jtraj',robot_name='allegro')
        self.viz=VizInHand(o_mesh,init_node=False)
    def fg_finger(self,finger,js,des_cpt,TRAJ=False):
        j_traj,cpt=self.fg_client.call(des_cpt,finger,js)
        # get full trajectory?
        if(TRAJ):
            print 'Getting full trajectory'
            
        return j_traj,cpt
    def in_grasp(self,init_js,obj_pose,des_obj_pose):

        j_traj,n_obj_pose,n_js=self.ig_client.call_server(init_js,obj_pose.pose,des_obj_pose.pose)
        
        return n_obj_pose,n_js,j_traj
    
class pickleData:
    def read_file(self,file_name):
        file_ = open(file_name,'rb')
        obj=pickle.load(file_)
        return obj
    
    def get_config(self,obj,i):
        obj_pose=obj[i][0]
        js=obj[i][1]
        cpt=obj[i][3]
        des_contact_pt=obj[i][2]
        return obj_pose,js,cpt,des_contact_pt

class gaitHelper:
    def get_T_mat(self,pose):
        T=np.eye(4)
        T[0:3,3]=np.ravel([pose.position.x,pose.position.y,pose.position.z])
        #
        R=PyKDL.Rotation()
        R=R.Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        for k in range(3):
            for l in range(3):
                T[k,l]=R[k,l]
                
        return T
    def get_T_mat_pt(self,pt):
        T=np.eye(4)
        T[0:3,3]=np.ravel([pt.x,pt.y,pt.z])
        return T

    def get_cpt(self,obj_pose,cpts,new_obj_pose):
        # given contact points in current object frame, transform it to new_object_pose
        o_T_c=[]
        obj_mat=np.matrix(inv(self.get_T_mat(new_obj_pose.pose)))
        z_obj_mat=np.matrix(inv(self.get_T_mat(obj_pose.pose)))
        n_cpts=copy.deepcopy(cpts)
        for i in range(len(n_cpts)):
            pt=cpts[i]
            T_c=self.get_T_mat_pt(pt.point)
            o_T_c.append(obj_mat*T_c)
            new_pt=np.ravel((inv(z_obj_mat)*np.matrix(o_T_c[i]))[0:3,3])
            pt.point.x=new_pt[0]
            pt.point.y=new_pt[1]
            pt.point.z=new_pt[2]
            n_cpts[i]=pt
        return n_cpts
    def rigid_translate(self,c_pts,d_pts,obj_pose):
        curr_obj_pose=np.zeros(7)
        curr_obj_pose[0]=obj_pose.pose.position.x
        curr_obj_pose[1]=obj_pose.pose.position.y
        curr_obj_pose[2]=obj_pose.pose.position.z
        curr_obj_pose[3]=obj_pose.pose.orientation.x
        curr_obj_pose[4]=obj_pose.pose.orientation.y
        curr_obj_pose[5]=obj_pose.pose.orientation.z
        curr_obj_pose[6]=obj_pose.pose.orientation.w

        cpts=[]
        dpts=[]
        for i in range(len(c_pts)):
            cpts.append(np.array([c_pts[i].point.x, c_pts[i].point.y, c_pts[i].point.z]))
            dpts.append(np.array([d_pts[i].point.x, d_pts[i].point.y, d_pts[i].point.z]))
    
    
            
        delta_t=np.zeros(3)
        for i in range(len(dpts)):
            delta_t+=dpts[i]-cpts[i]
            
        new_d_pose=delta_t
        sca=0.08
        d_obj_pose=copy.deepcopy(obj_pose)
        d_obj_pose.pose.position.x=curr_obj_pose[0]-new_d_pose[0]*sca
        d_obj_pose.pose.position.y=curr_obj_pose[1]-new_d_pose[1]*sca
        d_obj_pose.pose.position.z=curr_obj_pose[2]-new_d_pose[2]*sca
        d_obj_pose.pose.orientation.x=curr_obj_pose[3]
        d_obj_pose.pose.orientation.y=curr_obj_pose[4]
        d_obj_pose.pose.orientation.z=curr_obj_pose[5]
        d_obj_pose.pose.orientation.w=curr_obj_pose[6]
        return d_obj_pose


    def rigid_transform(self,c_pts,d_pts,obj_pose):
        curr_obj_pose=np.zeros(7)
        curr_obj_pose[0]=obj_pose.pose.position.x
        curr_obj_pose[1]=obj_pose.pose.position.y
        curr_obj_pose[2]=obj_pose.pose.position.z
        curr_obj_pose[3]=obj_pose.pose.orientation.x
        curr_obj_pose[4]=obj_pose.pose.orientation.y
        curr_obj_pose[5]=obj_pose.pose.orientation.z
        curr_obj_pose[6]=obj_pose.pose.orientation.w
        
        cpts=[]
        dpts=[]
        for i in range(len(c_pts)):
            cpts.append(np.array([c_pts[i].point.x, c_pts[i].point.y, c_pts[i].point.z]))
            dpts.append(np.array([d_pts[i].point.x, d_pts[i].point.y, d_pts[i].point.z]))
        frm=frames.transformMat()
        # Find transform from obj_pose to minimize error between c_pts and d_pts
        n_pose_mat=frm.rigid_svd(dpts,cpts,True)
        n_mat=np.matrix(frm.T_pose(curr_obj_pose))*np.matrix(n_pose_mat)
        
        n_pose=frm.pose_T(n_mat,False)
        #
        new_d_pose=n_pose-curr_obj_pose
        sca=0.08
        d_obj_pose=copy.deepcopy(obj_pose)
        d_obj_pose.pose.position.x=curr_obj_pose[0]+new_d_pose[0]*sca
        d_obj_pose.pose.position.y=curr_obj_pose[1]+new_d_pose[1]*sca
        d_obj_pose.pose.position.z=curr_obj_pose[2]+new_d_pose[2]*sca
        d_obj_pose.pose.orientation.x=n_pose[3]
        d_obj_pose.pose.orientation.y=n_pose[4]
        d_obj_pose.pose.orientation.z=n_pose[5]
        d_obj_pose.pose.orientation.w=n_pose[6]
        return d_obj_pose




if __name__=='__main__':
    gait_help=gaitHelper()
    p_data=pickleData()
    obj_file='../python/r_pringles_0.p'

    obj_dat=p_data.read_file(obj_file)
    obj_pose,init_js,cpt,_=p_data.get_config(obj_dat,0)
    des_obj_pose=obj_dat[-2][0]
    
    des_cpt=obj_dat[-2][2]
    # load pickle file:
    #print obj_pose,js,cpt,des_cpt
    

    # load object mesh and pose:
    mesh_f='/home/bala/object_models/pringles/vertices.ply'
    mesh_stl='file:///home/bala/object_models/pringles/nontextured.stl'

    #mesh_f="/home/bala/catkin_ws/src/urlg_robots_description/meshes/allegro/convex_hull/vertices/index_link_1.ply"
    pose=obj_pose.pose
    gg=graspGait(mesh_stl,weights=[0.001,10.0,10.0,0.0,100.0])

    gg.viz.viz_obj_mesh(obj_pose,a=1.0)
    gg.viz.viz_cpts(cpt)
    
    # transform desired points to initial obj pose:
    d_pts=gait_help.get_cpt(obj_pose,des_cpt,des_obj_pose)
    gg.viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])

    # intial grasp vis:
    i_jtraj=JointTrajectory()
    
    
    
    # move to initial grasp
    j_initial=init_js.position
    js=JointState()
    js.position=np.array(j_initial)
    #raw_input('next finger?')
    
    for t in range(10):
        raw_input('Next timestep?')
        
        gg.fg_client.update_obj(obj_pose.pose,mesh_f)
    
        for i in range(4):
            des_contact_pt=d_pts[i].point
            jtraj,cpt[i]=gg.fg_finger(i,js,des_contact_pt)
            gg.viz.viz_cpts(cpt)
            
            # visualize jtraj:
            jtraj.points[0]=jtraj.points[-1]
            gg.traj_server.viz_joint_traj(jtraj,base_link='allegro_mount')
            js.position=jtraj.points[-1].positions
            raw_input('Next finger?')

        exit()
        # get rigid transform to minimize distance:
        d_obj_pose=gait_help.rigid_translate(cpt,d_pts,obj_pose)

        gg.viz.viz_obj_mesh(d_obj_pose,t='n_obj_pose',rgb=[0,1,0],a=0.5)

        # call in-grasp:
        n_obj_pose,n_js,jtraj=gg.in_grasp(js,obj_pose,d_obj_pose)
    
        gg.viz.viz_obj_mesh(n_obj_pose,t='ig_obj_pose',rgb=[1,0,0],a=0.5)
        
        # visualize jtraj:
        jtraj.points[0]=jtraj.points[-1]
        gg.traj_server.viz_joint_traj(jtraj,base_link='allegro_mount')

        # update contact points:
        d_pts=gait_help.get_cpt(n_obj_pose,d_pts,obj_pose)
        cpt=gait_help.get_cpt(n_obj_pose,cpt,obj_pose)
        gg.viz.viz_cpts(cpt)
        gg.viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])
        obj_pose=copy.deepcopy(n_obj_pose)
        # update obj pose:
        gg.viz.viz_obj_mesh(obj_pose,a=0.9)
        js.position=jtraj.points[-1].positions

