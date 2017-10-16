from in_grasp_client import inGraspClient
import sys
import time
import tf_api
import copy
import pickle
# Find package with allegro model
from rospkg import RosPack
import subprocess
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

path_allegro=rp.get_path('allegro_driver')+'/scripts'
sys.path.insert(0,path_allegro)
from allegro_ros_api import handAPI

class inGraspPipeline:
    def __init__(self,publish_prefix='/allegro_hand_right',listen_prefix='/allegro_hand_right', init_node=False, node_name='in_grasp_pipeline',weights=[0.001,10.0,1.0,0.1,100.0]):
        if(init_node):
            rospy.init_node(node_name)
            
        self.allegro_drv=handAPI(init_node=False,publish_prefix=publish_prefix,listen_prefix=listen_prefix)
        self.in_grasp=inGraspClient()
        self.weights=weights
        self.tf_api=tf_api.tfAPI()
    def read_objects(self, file_name="../data/trajectories/exp_2_goal_poses.csv"):
        obj_list=ObjectList(file_name,' ')
        aruco_file_="../data/initial_data/object_grasps/object_centroids.csv"
        obj_list.add_aruco_centroid_locations(aruco_file_)

        self.obj_list=obj_list

    def generate_traj(self,idx,folder="../data/trajectories/journal/open_loop/"):
        obj_=self.obj_list.arr[idx]
        j0=obj_.grasp
        obj_pose=obj_.initial_pose
        des_obj_pose=obj_.des
        fingers=obj_.fing

        # Get plan from in_grasp client:
        weights=self.weights
        f_j=j0.copy()-0.05
        f_j[8:12]=j0[8:12]
        start = time.time()
        j_traj,n_pose=self.in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=2,final_joints=f_j)


        weights[-1]=0.2# RR_orient, RR_pos, T, interp, vel limits
        f_j=j_traj.points[-1].positions
        start = time.time()
        j_traj,n_pose=self.in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=10,final_joints=f_j)
        
        # save trajectory
        folder+obj_.f_name
        pickle.dump( j_traj, open( folder+obj_.f_name+".p", "wb" ) )
        
    def write(self, idx, file_name):
        print "TODO"

    def rhc_traj(self, idx, T=10, OBJ_TRACKING=False, FROM_FILE=True, folder="../data/trajectories/journal/open_loop/"):
        # Receding horizon control:
        obj_=self.obj_list.arr[idx]
        j0=obj_.grasp
        obj_pose=obj_.initial_pose
        des_obj_pose=obj_.des
        fingers=obj_.fing
        
        print 'Object: '+obj_.name
        # Enable grav comp mode
        #self.allegro_drv.engage_grav_comp()

        
        # Get plan from in_grasp client:
        weights=self.weights
        if(FROM_FILE==False):
            f_j=j0.copy()-0.05
            f_j[8:12]=j0[8:12]
            start = time.time()
            j_traj,n_pose=self.in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=2,final_joints=f_j)
            end = time.time()
            print(end - start)
            weights[-1]=0.2# RR_orient, RR_pos, T, interp, vel limits
            f_j=j_traj.points[-1].positions
            start = time.time()
            j_traj,n_pose=self.in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=T,final_joints=f_j)
            end = time.time()
            print(end-start)
        else:
            j_traj=pickle.load( open( folder+obj_.f_name+'.p', "rb" ) )
            
        print j_traj
        exit()
        # Move to initial grasp
        raw_input('Move to initial grasp?')
        self.allegro_drv.publish_joints(j0)
        if(OBJ_TRACKING):
            P_=[] # list of subprocesses
            raw_input('Placed Object?')
            P_.append(self.tf_api.tf_pose_pub_process(obj_.initial_pose,'i') )
            P_.append(self.tf_api.tf_pose_pub_process(obj_.offset,'rigid','thumb_tip'))
            P_.append(self.tf_api.tf_pose_pub_process(obj_.des,'G'))
            
            P_.append(self.tf_api.tf_pose_pub_process(obj_.aruco,'obj_ar','ar') )

            raw_input('Placed Object and Aligned initial frames?')
            initial_frame=self.tf_api.tf_find_transform('/i','/obj_ar')
            #palm_rotation=allegroStates.tf_find_transform('/palm_link','/obj_ar')
            P_.append(self.tf_api.tf_pose_pub_process(initial_frame,'/obj','/obj_ar'))
            time.sleep(0.5)
            
        # Run trajectory
        raw_input("Run Receding Horizon control?")

        #
        t=0
        H=10
        while(t<T-2):
            # send first 10 timesteps:
            u_curr=self.allegro_drv.interpolate_joint_traj(j_traj)

            self.allegro_drv.publish_joint_arr(u_curr[0:H])

            # Update current state
            curr_obj_pose=self.tf_api.tf_find_transform('obj')
            j0=self.allegro_drv.joint_state.position
            
            # remove first H steps from trajectory
            j_traj.points.pop(0)
            t+=1
            # replan trajectory:
            j_traj,n_pose=self.in_grasp.call_server_list(j0, curr_obj_pose, des_obj_pose, weights, fingers, T=T-t, final_joints=u_curr[-1],initial_j_traj=j_traj)
            print len(j_traj.points)

        self.allegro_drv.publish_joint_arr(u_curr[H:])
        #self.allegro_drv.publish_joint_traj(j_traj)

        # Collect data
        suc=raw_input('Press Enter - store final pose \n Press F to store as failure ')
    
        if(suc==''):
            # Get object pose from aruco marker
            
            # Get final joint angles
            #joint_angles=allegroStates.joint_states.position
            # Get final object pose
            reached_pose=self.tf_api.tf_find_transform('obj')
            obj_.reached=reached_pose
            print reached_pose[0:3]
        else:
            # Store failure
            obj_.reached='F'
        #objects.write_reached_pose(folder+trial_name)
        
        for i in P_:
            i.send_signal(subprocess.signal.SIGINT)

        # Enable grav comp mode
        #raw_input("grav comp?")
        #self.allegro_drv.engage_grav_comp()

        
    def run_traj(self,idx, OBJ_TRACKING=False, RECORD_DATA=False):
        # Read idx from file
        obj_=self.obj_list.arr[idx]
        j0=obj_.grasp
        obj_pose=obj_.initial_pose
        des_obj_pose=obj_.des
        fingers=obj_.fing
        
        print 'Object: '+obj_.name
        # Enable grav comp mode
        #self.allegro_drv.engage_grav_comp()

        
        # Get plan from in_grasp client:
        weights=self.weights
        f_j=j0.copy()-0.05
        f_j[8:12]=j0[8:12]
        start = time.time()
        j_traj,n_pose=self.in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=2,final_joints=f_j)
        end = time.time()
        print(end - start)
        weights[-1]=0.2# RR_orient, RR_pos, T, interp, vel limits
        f_j=j_traj.points[-1].positions
        start = time.time()
        j_traj,n_pose=self.in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=10,final_joints=f_j)
        end = time.time()
        print(end-start)
        # Move to initial grasp
        raw_input('Move to initial grasp?')
        self.allegro_drv.publish_joints(j0)
        if(OBJ_TRACKING):
            P_=[] # list of subprocesses
            raw_input('Placed Object?')
            P_.append(self.tf_api.tf_pose_pub_process(obj_.initial_pose,'i') )
            P_.append(self.tf_api.tf_pose_pub_process(obj_.offset,'rigid','thumb_tip'))
            P_.append(self.tf_api.tf_pose_pub_process(obj_.des,'G'))
            
            P_.append(self.tf_api.tf_pose_pub_process(obj_.aruco,'obj_ar','ar') )

            raw_input('Placed Object and Aligned initial frames?')
            initial_frame=self.tf_api.tf_find_transform('/i','/obj_ar')
            #palm_rotation=allegroStates.tf_find_transform('/palm_link','/obj_ar')
            P_.append(self.tf_api.tf_pose_pub_process(initial_frame,'/obj','/obj_ar'))
            time.sleep(0.5)
            
        # Run trajectory
        raw_input("Run trajectory?")
        self.allegro_drv.publish_joint_traj(j_traj)

        # Collect data
        suc=raw_input('Press Enter - store final pose \n Press F to store as failure ')
    
        if(suc==''):
            # Get object pose from aruco marker
            
            # Get final joint angles
            #joint_angles=allegroStates.joint_states.position
            # Get final object pose
            reached_pose=self.tf_api.tf_find_transform('obj')
            obj_.reached=reached_pose
            print reached_pose[0:3]
        else:
            # Store failure
            obj_.reached='F'
        #objects.write_reached_pose(folder+trial_name)
        
        for i in P_:
            i.send_signal(subprocess.signal.SIGINT)

        # Enable grav comp mode
        raw_input("grav comp?")

        self.allegro_drv.engage_grav_comp()

def get_trajectories():
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()
    start=time.time()
    for i in range(len(in_grasp_pip.obj_list.arr)):
        print i
        in_grasp_pip.generate_traj(i)
        
    end = time.time()
    print(end-start)
    
if __name__=='__main__':
    #in_grasp_pip=inGraspPipeline(init_node=True,publish_prefix='/allegroHand_0',listen_prefix='/allegroHand_0')
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()
    in_grasp_pip.rhc_traj(int(sys.argv[1]),OBJ_TRACKING=True,FROM_FILE=True)
        
def demo():
    file_="../data/trajectories/exp_2_goal_poses.csv"

    obj_list=ObjectList(file_,' ')
    obj_=obj_list.arr[int(sys.argv[1])]

    j0=obj_.grasp
    obj_pose=obj_.initial_pose
    des_obj_pose=obj_.des
    fingers=obj_.fing
    print obj_.name

    # Initialize allegro api:
    #allegro_drv=handAPI(publish_prefix='/allegroHand_0',listen_prefix='/allegroHand_0',init_node=True)
    allegro_drv=handAPI(init_node=True)
    # Initialize in-grasp client:
    in_grasp=inGraspClient()
    #robot_kdl=allegroKinematics(fingers=[0,1,2,3])
    #des_obj_pose=robot_kdl.end_effector_pose_array(np.array([0.5,0.5,0.5,0.5]),3)
    #print des_obj_pose
    weights=[0.01,10.0,1.0,0.1,100.0]# RR_orient, RR_pos, T, interp, vel limits


    
    # send initial grasp:
    raw_input("Initial grasp?")
    allegro_drv.publish_joints(j0)
    #exit()
    # Compute joint traj:
    f_j=j0.copy()-0.05
    f_j[8:12]=j0[8:12]
    start = time.time()
    j_traj,n_pose=in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=2,final_joints=f_j)
    end = time.time()
    print(end - start)
    weights[-1]=0.2# RR_orient, RR_pos, T, interp, vel limits
    f_j=j_traj.points[-1].positions
    start = time.time()
    j_traj,n_pose=in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=10,final_joints=f_j)
    end = time.time()
    print(end-start)
    # interpolate joint traj:
    
    # send joint trajectory to robot:
    raw_input("send traj?")
    allegro_drv.publish_joint_traj(j_traj)

    
## Single pipeline:

# Move to initial grasp

# visualize object in initial pose

# Place object

# Track object

# Get offsets

# Get in_grasp plan

# visualize plan

# execute trajectory

# Get final object pose


## Replanning pipeline:

# MPC:
