
from in_grasp_client import inGraspClient
import sys
import time
import tf_api
import copy
import pickle
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from in_hand_manipulation_msgs.srv import RunMPC
from data_record_api import rosbagAPI
import pandas as pd
from subprocess import Popen
import subprocess
import os
import signal

from sensor_msgs.msg import JointState
# Find package with allegro model
from rospkg import RosPack
import subprocess
import tf
# import required allegro model files
rp=RosPack()
rp.list()
path_in_hand_data=rp.get_path('in_hand_manipulation_data')+'/data/in_grasp_manipulation'
path_in_hand_data_scripts=rp.get_path('in_hand_manipulation_data')+'/scripts'
sys.path.insert(0,path_in_hand_data_scripts)
from error_api import errorAPI
from data_api import pdAPI
path_in_grasp=rp.get_path('in_grasp_manipulation')+'/python'
sys.path.insert(0,path_in_grasp)
import viz_markers
from object_list import *
path_=rp.get_path('trajectory_smoothing')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *

path=rp.get_path('ll4ma_kdl')+'/scripts'
sys.path.insert(0,path)
from kdl_allegro_model import allegroKinematics

path_allegro=rp.get_path('allegro_driver')+'/scripts'
sys.path.insert(0,path_allegro)
from allegro_ros_api import handAPI

path_in_hand=rp.get_path('in_hand_manipulation')+'/scripts'
sys.path.insert(0,path_in_hand)
from hand_ik_client import handIKClient

#path_grasp_gaits=rp.get_path('grasp_gaits')+'/scripts'
#sys.path.insert(0,path_grasp_gaits)
#from viz_markers import VizInHand
class pdData:
    def __init__(self):
        print 'pdData initialized'
    def read(self,f_name):
        df=pd.read_csv(f_name)
        self.df=df
    def write_data(self,obj_,idx,t_no,f_name):
        # update pd data frame and write to file
        des_pose=' '.join(map(str, obj_.des))
        reach_pose=' '.join(map(str, obj_.reached))
        df_new=pd.DataFrame([[obj_.name,idx,t_no,des_pose,reach_pose]],columns=list(self.df.columns))
        df_=self.df.append(df_new)
        #print df_new
        df_.to_csv(f_name,index=False)

class inGraspData:
    def __init__(self,data_root='',publish_prefix='',listen_prefix='',weights=[0.001,0.5,100.0,1.0,0.4]):
        self.data_root=data_root
        self.weights=weights
        self.pose_api=tf_api.poseAPI()
        self.allegro_drv=handAPI(init_node=False,publish_prefix=publish_prefix,listen_prefix=listen_prefix)
        self.error_api=errorAPI()
        self.obj_list=None
        self.in_grasp=inGraspClient()
        self.traj_server=trajectoryServer(10,init_node=False,topic_name='/in_grasp/jtraj',robot_name='allegro')

        print "initialized data recorder"
    def read_objects(self,file_name):
        obj_list=ObjectList(self.data_root+file_name,' ')
        aruco_file_=self.data_root+"/initial_data/object_grasps/object_centroids.csv"
        obj_list.add_aruco_centroid_locations(aruco_file_)
        self.obj_list=obj_list
    def generate_traj(self,idx):
        obj_=self.obj_list.arr[idx]
        j0=obj_.grasp
        js_j0=JointState()
        js_j0.position=j0.copy()
        obj_pose=obj_.initial_pose
        des_obj_pose=obj_.des
        #print des_obj_pose
        fingers=obj_.fing
        des_pose_=self.pose_api.pose_from_arr(self.obj_list.arr[idx].des).pose
        # Get ik for final object pose:
        # Compute initial grasp contact points and find offsets
        des_pose_arr=[]
        for i in range(4):
            if(i in fingers):
                f_pose=self.allegro_drv.robot_kdl.end_effector_pose_array(j0[i*4:i*4+4],i)
                des_pose=np.zeros(6)
                des_pose[0:3]=des_obj_pose[0:3]+f_pose[0:3]-obj_.initial_pose[0:3]
                des_pose_arr.append(self.pose_api.pose_from_arr(des_pose).pose)
            else:
                des_pose_arr.append(Pose())
        start=time.time()

        f_j=[]#j_traj.points[-1].positions
        #weights=[0.0,0.001,0.1,0.0,1.5]
        weights=copy.deepcopy(self.weights)
        weights[3]=0.0 # smooth acc
        weights[4]=10.5 # vel limits
        j_traj,n_pose,obj_traj=self.in_grasp.call_server_list(j0, obj_pose,des_obj_pose,weights, fingers,T=1, final_joints=f_j,srv_topic="/in_grasp_planner/get_plan")
        weights=copy.deepcopy(self.weights)

        j_traj,n_pose,obj_traj=self.in_grasp.call_server_list(j0, obj_pose,des_obj_pose,weights, fingers,T=10, final_joints=j_traj.points[-1].positions,srv_topic="/in_grasp_planner/get_plan")
        
        plan_time=time.time()-start
        print 'time:'+str(time.time()-start)
        self.traj_server.viz_joint_traj(j_traj,base_link='allegro_mount')
        # plot
        #self.traj_server.plot_j_acc_profile(j_traj,[12,13,14,15])
        #self.traj_server.viz_joint_traj(j_traj,base_link='allegro_mount')
        #self.pose_api.plot_acc_profiles([j_traj],['min_acc'])

        #return j_traj,obj_traj,obj_traj_rr,j_tra
        '''
        weights=copy.deepcopy(self.weights)
        f_j=j_traj.points[-1].positions
        init_traj=JointTrajectory()
        _,_,_,rr_metrics=self.in_grasp.call_server_list(j0, obj_pose,des_obj_pose,weights, fingers,T=10, final_joints=f_j,srv_topic='/in_hand/primitive/in_grasp_manipulation/metrics',initial_j_traj=j_traj)
        '''
        
        # metrics=[planning time, reached position error, orientation error, traj length(from initial to desired)]
        metrics=[]
        metrics.append(plan_time)


        
        #metrics.append(rr_metrics[0])
        # Compute desired pose cost:
        reach_obj_pose=np.zeros(7)
        reach_obj_pose[0]=obj_traj.poses[-1].position.x
        reach_obj_pose[1]=obj_traj.poses[-1].position.y
        reach_obj_pose[2]=obj_traj.poses[-1].position.z
        reach_obj_pose[3]=obj_traj.poses[-1].orientation.x
        reach_obj_pose[4]=obj_traj.poses[-1].orientation.y
        reach_obj_pose[5]=obj_traj.poses[-1].orientation.z
        reach_obj_pose[6]=obj_traj.poses[-1].orientation.w

        des_obj=np.zeros(7)
        des_obj[0:3]=des_obj_pose[0:3]
        des_obj[3:7]=self.pose_api.quat_from_rpy(des_obj_pose[3:6])
        p_err,o_err=self.error_api.pose_err(des_obj,reach_obj_pose)
        metrics.append(p_err)
        metrics.append(o_err)

        # compute traj length:
        initial_obj_pose=np.zeros(7)
        initial_obj_pose[0:3]=obj_pose[0:3]
        initial_obj_pose[3:7]=self.pose_api.quat_from_rpy(obj_pose[3:6])
        p_err,o_err=self.error_api.pose_err(initial_obj_pose,des_obj)
        metrics.append(p_err)
        metrics.append(o_err)

        '''
        if(WRITE_TO_FILE):
            # save trajectory
            folder+obj_.f_name
            pickle.dump( [j_traj,obj_traj], open( folder+obj_.f_name+".p", "wb" ) )
            return des_obj,reach_obj_pose,metrics
        '''
        return j_traj,obj_traj,metrics

if __name__=='__main__':
    rospy.init_node('generate')
    # file for goal poses
    f_name="/initial_data/goal_poses/relaxed-rigidity.csv"

    # weights: ro,rp,des,waypoint,velo
    ik=[]
    rp=[0.0,0.05,1.0,0.0,0.4]
    rpo=[0.001,0.05,1.0,0.0,0.4]
    rr=[0.001,0.05,1.0,0.001,0.4]
    in_grasp=inGraspData(data_root='../data/',weights=rp)
    in_grasp.read_objects(f_name)
    # read initial and goal poses

    # generate trajectory
    idx=0
    _,_,metrics=in_grasp.generate_traj(idx)
    #print metrics
    # write to file

    # save solution to data frame

    # write solution
