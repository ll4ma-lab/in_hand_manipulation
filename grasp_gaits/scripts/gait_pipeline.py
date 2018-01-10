#import rospy
from gait_client import *
import sys
from rospkg import RosPack
rp=RosPack()
path_=rp.get_path('optimization_pkg')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *

class graspGait:
    def __init__(self):
        self.fg_client=fgClient()
        self.traj_server=trajectoryServer(10,init_node=True,topic_name='/in_grasp/jtraj',robot_name='allegro')

    def fg_finger(self,finger,js,des_cpt):
        j_traj,_=self.fg_client.call(des_cpt,finger,js)
        return j_traj

if __name__=='__main__':
    gg=graspGait()

    # load object mesh and pose:
    mesh_f='/home/bala/object_models/mustard_bottle/google_16k/vertices.ply'
    #mesh_f="/home/bala/catkin_ws/src/urlg_robots_description/meshes/allegro/convex_hull/vertices/index_link_1.ply"
    pose=Pose()
    pose.position.x=0.1
    pose.position.z=0.0
    pose.orientation.w=1.0
    gg.fg_client.update_obj(pose,mesh_f)
    
    # move to initial grasp
    j_initial=[0.2,0.8,0.9,0.5,
               0.0,0.8,0.9,0.5,
               0.0,0.8,0.9,0.5,
               1.52,0.0,0.7,0.1]

    js=JointState()
    js.position=j_initial

    des_contact_pt=Point()
    des_contact_pt.x=0.1
    des_contact_pt.z=0.1
    des_contact_pt.y=-0.1
    
    jtraj=gg.fg_finger(0,js,des_contact_pt)

    # visualize jtraj:
    gg.traj_server.viz_joint_traj(jtraj,base_link='allegro_mount')
            
    # 
    # get initial grasp: object pose, contact points
    
    # get desired grasp: object pose, Contact points
