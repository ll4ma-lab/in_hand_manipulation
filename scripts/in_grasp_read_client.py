from in_grasp_client import inGraspClient
import sys
import time
# Find package with allegro model
from rospkg import RosPack

# import required allegro model files
rp=RosPack()
rp.list()

path_in_grasp=rp.get_path('in_grasp_manipulation')+'/python'
sys.path.insert(0,path_in_grasp)

from object_list import *
path_=rp.get_path('optimization_pkg')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *

if __name__=='__main__':
    file_="../data/trajectories/exp_2_goal_poses.csv"
    obj_list=ObjectList(file_,' ')
    obj_=obj_list.arr[0]

    j0=obj_.grasp
    obj_pose=obj_.initial_pose
    des_obj_pose=obj_.des
    fingers=obj_.fing
    
    
    in_grasp=inGraspClient()
    j_traj=in_grasp.call_server_list(j0,obj_pose,des_obj_pose,[1.0,0.0,0.0],fingers)
    np.set_printoptions(precision=2)
    # Publish joint traj:
    t_name='planner/in_grasp_traj'
    traj_server=trajectoryServer(100,robot_name='allegro',topic_name=t_name,init_node=True)
    time.sleep(1.0)
    traj_server.viz_joint_traj(j_traj,'allegro_mount')

    for i in range(len(j_traj.points)):
        print np.array(j_traj.points[i].positions)
