#import rospy
from gait_client import *
from grasp_client import *
import sys
from read_data import *
import trimesh
from rospkg import RosPack
rp=RosPack()
path_=rp.get_path('ll4ma_planner')+'/scripts'
sys.path.insert(0,path_)
from tf_helper import *
path_=rp.get_path('optimization_pkg')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *

path_=rp.get_path('grasp_gaits')+'/python'
sys.path.insert(0,path_)
from viz_markers import VizInHand
from geometry_msgs.msg import PoseStamped,PointStamped
import grasp_class
import pickle
import PyKDL
from numpy.linalg import inv
import copy
import frames
path_=rp.get_path('in_grasp_manipulation')+'/scripts'
sys.path.insert(0,path_)
from in_grasp_client import inGraspClient
path_allegro=rp.get_path('allegro_driver')+'/scripts'
sys.path.insert(0,path_allegro)
from allegro_ros_api import handAPI

class graspGait:
    def __init__(self,o_mesh,weights,hand_prefix='/allegro_hand_right'):
        self.fg_client=fgClient()
        self.grasp_client=graspClient()
        self.ig_client=inGraspClient(weights)
        
        self.traj_server=trajectoryServer(10,init_node=True,topic_name='/in_grasp/jtraj',robot_name='allegro')
        self.allegro_drv=handAPI(init_node=False,publish_prefix=hand_prefix,listen_prefix=hand_prefix)
                
        self.viz=VizInHand(o_mesh,init_node=False)
        self.obj=grasp_class.object(o_mesh[7:])

    def get_normal(self,contact_pt):
        dat=trimesh.proximity.nearby_faces(self.obj.mesh,[contact_pt])
        norm=self.obj.mesh.face_normals[dat[0][0]]
        return norm
    def fg_finger(self,finger,js,des_cpt,final_js,TRAJ=False,T=1,slide=False,normal=Point()):
        j_traj,cpt=self.fg_client.call(des_cpt,finger,js,final_js,T=T,slide=slide,contact_norm=normal)
        # get full trajectory?
        if(TRAJ):
            print 'Getting full trajectory'
            
        return j_traj,cpt
    
    def interpolate_jtraj(self,jtraj):
        pos_array=np.zeros((len(jtraj.points),16))
        for i in range(len(jtraj.points)):
            pos_array[i,:]=jtraj.points[i].positions
        t=len(pos_array)-1
        T=t*10
        dt=T/t
        #print dt
        new_traj=np.zeros((T,16))
        for k in range(len(pos_array[0,:])):
            new_traj[0:dt,k]=np.linspace(pos_array[0,k],pos_array[1,k],dt)

        for i in range(1,len(pos_array)-1):
            for k in range(len(pos_array[i,:])):
                new_traj[i*dt-1:i*dt+dt,k]=np.linspace(pos_array[i,k],pos_array[i+1,k],dt+1)
        new_jtraj=JointTrajectory()
        new_jtraj.header=jtraj.header
        new_jtraj.joint_names=jtraj.joint_names
        for i in range(len(new_traj)):
            pt=JointTrajectoryPoint()
            pt.positions=new_traj[i,:]
            new_jtraj.points.append(pt)
        return new_jtraj

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
    obj_file='mustard_0.p'
    gF=gaitFile()
    g_f=gF.read_file(obj_file)
    mesh_f=g_f[0].mesh_f
    mesh_stl=g_f[0].mesh_stl
    
    obj_pose=g_f[0].init_o_pose
    print obj_pose
    cpt=g_f[0].cpt
    d_pts=g_f[0].des_cpt
    
    d_pts=gait_help.get_cpt(obj_pose,d_pts,g_f[-1].final_o_pose)
    
    # transform desired points to initial object pose:
    
    gg=graspGait(mesh_stl,weights=[0.001,10.0,10.0,0.0,100.0])
    
    gg.viz.viz_obj_mesh(obj_pose,a=1.0)
    gg.viz.viz_cpts(cpt)
    
    # transform desired points to initial obj pose:
    
    gg.viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])

    
    
    init_js=g_f[0].i_js
    
    # move to initial grasp
    #j_initial=init_js.position
    js=init_js
    mesh_f='/home/bala/object_models/mustard_bottle/google_16k/nontextured.binvox'

    gg.fg_client.update_obj(obj_pose.pose,mesh_f)
    
    # visualize initial grasp:
    js=g_f[0].i_js
    
    jtraj=JointTrajectory()
    j_pt=JointTrajectoryPoint()
    j_pt.positions=js.position
    jtraj.joint_names=js.name
    jtraj.points.append(j_pt)
    gg.traj_server.viz_joint_traj(jtraj,base_link='allegro_mount')
    raw_input("updated mesh")

    f_order=[3,0,1,2]
    for i in f_order:
        if(g_f[i].ig==True):
            # perform ingrasp
            print "performing in-grasp"
            
            #d_obj_pose=gait_help.rigid_translate(cpt,d_pts,obj_pose)
        
            #n_obj_pose,n_js,jtraj=gg.in_grasp(js,obj_pose,d_obj_pose)
            
            #gg.viz.viz_obj_mesh(d_obj_pose,t='n_obj_pose',rgb=[0,1,0],a=0.5)
            #gg.viz.viz_obj_mesh(n_obj_pose,t='ig_obj_pose',rgb=[1,0,0],a=0.5)
            #jtraj.points[0]=jtraj.points[-1]
            #gg.traj_server.viz_joint_traj(jtraj,base_link='allegro_mount')

        else:
            print "performing finger gait",g_f[i].f
            
            js=g_f[i].i_js
            des_js=g_f[i].f_js
            '''
            if(i==1):
                js_1=np.array(g_f[i-1].i_js.position)
                js.position=np.array(js.position)
                js.position[g_f[i].f*4:g_f[i].f*4+4]=js_1[g_f[i].f*4:g_f[i].f*4+4
            '''
            #print js.position
            #print des_js.position
            des_contact_pt=d_pts[g_f[i].f].point

            # transform contact point to object frame:
            home_pose=PoseStamped()
            home_pose.pose.orientation.w=1.0
            
            o_pt=gait_help.get_cpt(home_pose,[copy.deepcopy(d_pts[g_f[i].f])],obj_pose)[0].point
            # get normal at contact point:
            pt=[o_pt.x,o_pt.y,o_pt.z]
            pt_n=gg.get_normal(pt)
            normal=Point()
            normal.x=pt_n[0]
            normal.y=pt_n[1]
            normal.z=pt_n[2]
            n_pts=PointStamped()
            n_pts.point=normal
            print normal
            # transform normal to object pose:
            n_pt=gait_help.get_cpt(obj_pose,[n_pts],home_pose)[0].point
            print n_pt
            # visualize contact point and normal:
            gg.viz.viz_contact_normal(des_contact_pt,n_pt)
            slide=False
            if(g_f[i].f==3):
                slide=True
            slide=False
            #print des_contact_pt
            jtraj,cpt[i]=gg.fg_finger(i,js,des_contact_pt,des_js,T=1,slide=slide,normal=normal)
            #print cpt[i]
            gg.viz.viz_cpts(cpt)
            g_f[i].traj=jtraj
            # save pickle file:
            #pickle.dump(g_f,open(obj_file,"wb"))
            
            # visualize jtraj:
            #jtraj.points[0]=jtraj.points[-1]
            gg.traj_server.viz_joint_traj(jtraj,base_link='allegro_mount')
            js.position=jtraj.points[-1].positions
            #print jtraj.points[-1].positions
            raw_input('Next finger?')

        '''
        # update contact points:
        d_pts=gait_help.get_cpt(n_obj_pose,d_pts,obj_pose)
        cpt=gait_help.get_cpt(n_obj_pose,cpt,obj_pose)
        gg.viz.viz_cpts(cpt)
        gg.viz.viz_cpts(d_pts,t='d_cpt',rgb=[0.0,1.0,0.0])
        obj_pose=copy.deepcopy(n_obj_pose)
        # update obj pose:
        gg.viz.viz_obj_mesh(obj_pose,a=0.9)
        js.position=jtraj.points[-1].positions
        '''
