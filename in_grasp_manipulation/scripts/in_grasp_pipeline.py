from in_grasp_client import inGraspClient
import sys
import time
import tf_api
import copy
import pickle
from geometry_msgs.msg import PoseArray, Pose
from in_hand_manipulation_msgs.srv import RunMPC
# Find package with allegro model
from rospkg import RosPack
import subprocess
# import required allegro model files
rp=RosPack()
rp.list()
path_in_hand_data=rp.get_path('in_hand_manipulation_data')+'/data/in_grasp_manipulation'

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

#path_grasp_gaits=rp.get_path('grasp_gaits')+'/scripts'
#sys.path.insert(0,path_grasp_gaits)
#from viz_markers import VizInHand

class inGraspPipeline:
    def __init__(self,publish_prefix='/allegro_hand_right',listen_prefix='/allegro_hand_right', init_node=False, node_name='in_grasp_pipeline',weights=[0.001,10.0,1.0,0.1,100.0]):
        if(init_node):
            rospy.init_node(node_name)
            
        self.allegro_drv=handAPI(init_node=False,publish_prefix=publish_prefix,listen_prefix=listen_prefix)
        self.in_grasp=inGraspClient()
        self.weights=weights
        self.tf_api=tf_api.tfAPI()
        self.pub = rospy.Publisher("/in_grasp/obj_traj", PoseArray)

    def read_objects(self, file_name=path_in_hand_data+"/trajectories/exp_2_goal_poses.csv"):
        obj_list=ObjectList(file_name,' ')
        aruco_file_=path_in_hand_data+"/initial_data/object_grasps/object_centroids.csv"
        obj_list.add_aruco_centroid_locations(aruco_file_)

        self.obj_list=obj_list

    def generate_traj(self,idx,folder=path_in_hand_data+"/trajectories/journal/open_loop/",WRITE_TO_FILE=True):
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
        j_traj,n_pose,obj_traj=self.in_grasp.call_server_list(j0, obj_pose, des_obj_pose, weights, fingers, T=2, final_joints=f_j)


        weights[-1]=0.2# RR_orient, RR_pos, T, interp, vel limits
        f_j=j_traj.points[-1].positions
        start = time.time()
        j_traj,n_pose,obj_traj=self.in_grasp.call_server_list(j0,obj_pose,des_obj_pose,weights,fingers,T=10,final_joints=f_j)
        if(WRITE_TO_FILE):
            # save trajectory
            folder+obj_.f_name
            pickle.dump( j_traj, open( folder+obj_.f_name+".p", "wb" ) )
        return j_traj,obj_traj,n_pose
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
def vis_traj():
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()
    j_traj,obj_traj,obj_pose=in_grasp_pip.generate_traj(int(sys.argv[1]),WRITE_TO_FILE=False)
    r = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        in_grasp_pip.pub.publish(obj_traj)
        r.sleep()
    #print obj_traj
#def run_mpc():
    # Read initial trajectory:

def run_mpc():
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()
    j_traj,obj_traj,obj_pose=in_grasp_pip.generate_traj(int(sys.argv[1]),WRITE_TO_FILE=False)
    # call mpc service:
    rospy.wait_for_service('/in_grasp_manipulation/one_step_mpc')
    run_mpc=rospy.ServiceProxy('/in_grasp_manipulation/one_step_mpc',RunMPC)
    result_=run_mpc(obj_traj,j_traj)
        

    #j_traj=get_in_grasp_plan(j0,obj_pose,des_obj_pose,weights,fingers,rig_finger,T,j_final,initial_j_traj)

    
if __name__=='__main__':
    #vis_traj()
    run_mpc()
    '''
    #in_grasp_pip=inGraspPipeline(init_node=True,publish_prefix='/allegroHand_0',listen_prefix='/allegroHand_0')
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()
    in_grasp_pip.rhc_traj(int(sys.argv[1]),OBJ_TRACKING=True,FROM_FILE=True)
    '''
