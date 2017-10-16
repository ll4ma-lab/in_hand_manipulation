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

path=rp.get_path('ll4ma_kdl')+'/scripts'
sys.path.insert(0,path)
from kdl_allegro_model import allegroKinematics

if __name__=='__main__':
    file_="../data/trajectories/exp_2_goal_poses.csv"
    obj_list=ObjectList(file_,' ')
    obj_=obj_list.arr[int(sys.argv[1])]

    j0=obj_.grasp
    obj_pose=obj_.initial_pose
    des_obj_pose=obj_.des
    fingers=obj_.fing
    print obj_.name
    #print fingers
    
    in_grasp=inGraspClient()
    robot_kdl=allegroKinematics(fingers=[0,1,2,3])
    #des_obj_pose=robot_kdl.end_effector_pose_array(np.array([0.5,0.5,0.5,0.5]),3)
    #print des_obj_pose
    weights=[0.01,10.0,1.0,0.1,100.0]# RR_orient, RR_pos, T, interp, vel limits
    
    t_name='planner/in_grasp_traj'
    traj_server=trajectoryServer(100,robot_name='allegro',topic_name=t_name,init_node=True)
    time.sleep(1.0)

    f_j=j0.copy()-0.05
    f_j[8:12]=j0[8:12]
    start = time.time()
    j_traj,n_pose=in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=2,final_joints=f_j)
    end = time.time()
    print(end - start)
    traj_server.viz_joint_traj(j_traj,'allegro_mount')

    weights[-1]=0.2# RR_orient, RR_pos, T, interp, vel limits
    f_j=j_traj.points[-1].positions
    start = time.time()
    j_traj,n_pose=in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=10,final_joints=f_j)
    end = time.time()
    print(end - start)
    traj_server.viz_joint_traj(j_traj,'allegro_mount')
    exit()
    k=0
    while(k<3):
        f_j=j_traj.points[-1].positions
        obj_pose[0]+=0.001
        obj_pose[1]-=0.001
        obj_pose[2]+=0.001
        #obj_pose[3]+=0.0001
        start = time.time()
        j_traj,n_pose=in_grasp.call_server_list(j0, obj_pose, des_obj_pose, weights, fingers, T=10, final_joints=f_j, initial_j_traj=j_traj)
        end = time.time()
        print str(1/(end - start))
        k+=1
    traj_server.viz_joint_traj(j_traj,'allegro_mount')

    #print des_obj_pose
    #print n_pose
    #print j_traj
    np.set_printoptions(precision=2)

    # Publish joint traj:
    
    #for i in range(len(j_traj.points)):
    #   print np.array(j_traj.points[i].positions)
