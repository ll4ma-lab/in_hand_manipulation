#import rospy
from gait_client import *
import sys
from rospkg import RosPack
rp=RosPack()
path_=rp.get_path('optimization_pkg')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *


import pickle


class graspGait:
    def __init__(self):
        self.fg_client=fgClient()
        self.traj_server=trajectoryServer(10,init_node=True,topic_name='/in_grasp/jtraj',robot_name='allegro')

    def fg_finger(self,finger,js,des_cpt):
        j_traj,_=self.fg_client.call(des_cpt,finger,js)
        return j_traj

class pickleData:
    def read_file(self,file_name):
        file_ = open(file_name,'rb')
        obj=pickle.load(file_)
        return obj
    
    def get_initial_config(self,obj):
        obj_pose=obj[0][0]
        js=obj[0][1]
        cpt=obj[0][3]
        des_contact_pt=obj[0][2]
        return obj_pose,js,cpt,des_contact_pt

if __name__=='__main__':

    p_data=pickleData()
    obj_file='../python/r_mustard_0.p'

    obj_dat=p_data.read_file(obj_file)
    obj_pose,init_js,cpt,des_cpt=p_data.get_initial_config(obj_dat)
    # load pickle file:
    #print obj_pose,js,cpt,des_cpt
    
    gg=graspGait()

    # load object mesh and pose:
    mesh_f='/home/bala/object_models/mustard_bottle/google_16k/vertices.ply'
    #mesh_f="/home/bala/catkin_ws/src/urlg_robots_description/meshes/allegro/convex_hull/vertices/index_link_1.ply"
    pose=obj_pose.pose
    gg.fg_client.update_obj(pose,mesh_f)
    #exit()
    # move to initial grasp
    j_initial=init_js.position

    js=JointState()
    js.position=np.array(j_initial)
    #js.position[2]=0.0
    des_contact_pt=des_cpt[0].point
    
    jtraj=gg.fg_finger(0,js,des_contact_pt)

    # visualize jtraj:
    gg.traj_server.viz_joint_traj(jtraj,base_link='allegro_mount')
            
    # 
    # get initial grasp: object pose, contact points
    
    # get desired grasp: object pose, Contact points
