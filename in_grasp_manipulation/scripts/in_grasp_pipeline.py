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
        
class inGraspPipeline:
    def __init__(self,publish_prefix='/allegro_hand_right',listen_prefix='/allegro_hand_right', init_node=False, node_name='in_grasp_pipeline',weights=[0.01,100.0,1.0,0.1,0.4]):
        if(init_node):
            rospy.init_node(node_name)
        # weights=[RR_orient, RR_position, T_des, interpo]
        # weights=[0.05,100.0,
        self.allegro_drv=handAPI(init_node=False,publish_prefix=publish_prefix,listen_prefix=listen_prefix)
        self.in_grasp=inGraspClient()
        self.weights=weights
        self.tf_api=tf_api.tfAPI()
        self.bag_api=rosbagAPI()
        self.pub = rospy.Publisher("/in_grasp/obj_traj", PoseArray,queue_size=1)
        self.pub_rr = rospy.Publisher("/in_grasp/obj_traj_rr", PoseArray,queue_size=1)
        self.des_pub=rospy.Publisher("/in_grasp/des_obj_pose",PoseStamped,queue_size=1)
        self.traj_server=trajectoryServer(10,init_node=False,topic_name='/in_grasp/jtraj',robot_name='allegro')
        self.pose_api=tf_api.poseAPI()
        self.ik_server=handIKClient()
        self.error_api=errorAPI()
        self.pd_api=pdAPI()
        #self.kdl=allegroKinematics()
        #self.j_pub= rospy.Publisher("/in_grasp/joint_traj", JointTrajectory)
    def read_objects(self, file_name=path_in_hand_data+"/trajectories/waypoint_goal_poses.csv"):
        obj_list=ObjectList(file_name,' ')
        aruco_file_=path_in_hand_data+"/initial_data/object_grasps/object_centroids.csv"
        obj_list.add_aruco_centroid_locations(aruco_file_)

        self.obj_list=obj_list

    def generate_traj(self,idx,folder=path_in_hand_data+"/trajectories/journal/waypoint/",WRITE_TO_FILE=True):
        start=time.time()
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


        
        # IK
        j_init,j_traj=self.ik_server.call_server(js_j0,des_pose_arr,fingers)
        f_j=j_traj.points[-1].positions

        # RR IK
        weights=copy.deepcopy(self.weights)
        weights[-1]=100.0
        f_j=j_traj.points[-1].positions
        j_traj,n_pose,obj_traj=self.in_grasp.call_server_list(j0, obj_pose, des_obj_pose, weights, fingers, T=1, final_joints=f_j,initialize_opt=True)

        '''
        # RR obj interpol
        
        weights=copy.deepcopy(self.weights)
        f_j=j_traj.points[-1].positions
        j_traj,n_pose,obj_traj=self.in_grasp.call_server_list(j0, obj_pose,des_obj_pose,weights, fingers,T=10, final_joints=f_j)

        '''
        # RR min acc
        weights=copy.deepcopy(self.weights)
        
        j_traj,n_pose,obj_traj=self.in_grasp.call_server_list(j0, obj_pose,des_obj_pose,weights, fingers,T=10, final_joints=f_j,srv_topic='/in_hand/primitive/in_grasp_min_acc')


        
        plan_time=time.time()-start
        print 'time:'+str(time.time()-start)
        weights=copy.deepcopy(self.weights)
        f_j=j_traj.points[-1].positions
        init_traj=j_traj
        _,_,_,rr_metrics=self.in_grasp.call_server_list(j0, obj_pose,des_obj_pose,weights, fingers,T=10, final_joints=f_j,srv_topic='/in_hand/primitive/in_grasp_manipulation/metrics',initial_j_traj=j_traj)

        
        # metrics=[planning time, RR cost, reached position error, orientation error]
        metrics=[]
        metrics.append(plan_time)
        metrics.append(rr_metrics[0])
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
        #print metrics
        if(WRITE_TO_FILE):
            # save trajectory
            folder+obj_.f_name
            pickle.dump( [j_traj,obj_traj], open( folder+obj_.f_name+".p", "wb" ) )
            return des_obj,reach_obj_pose,metrics
        return j_traj,obj_traj,metrics
    def write(self, idx, file_name):
        print "TODO"

    def interpolate_jtraj(self,jtraj):
        pos_array=np.zeros((len(jtraj.points),16))
        for i in range(len(jtraj.points)):
            pos_array[i,:]=jtraj.points[i].positions
        t=len(pos_array)-1
        T=t*10
        dt=T/t
        print dt
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

    def lerp(self,q0,q1,h):
        q_interp=q0*(1-h)+q1*h

        q_norm=q_interp/np.linalg.norm(q_interp)
        return q_interp
    def interpolate_obj_traj(self,obj_traj):
        pos_arr=np.zeros((len(obj_traj.poses),3))
        quat_arr=np.zeros((len(obj_traj.poses),4))
        
        for i in range(len(obj_traj.poses)):
            pos_arr[i,:]=np.array([obj_traj.poses[i].position.x, obj_traj.poses[i].position.y, obj_traj.poses[i].position.z])
            quat_arr[i,:]=np.array([obj_traj.poses[i].orientation.x, obj_traj.poses[i].orientation.y, obj_traj.poses[i].orientation.z,obj_traj.poses[i].orientation.w])

        t=len(pos_arr)-1
        T=t*10
        dt=T/t
        new_pos_traj=np.zeros((T,3))

        new_quat_traj=np.zeros((T,4))
        # position interpolation:
        for k in range(len(pos_arr[0,:])):
            new_pos_traj[0:dt,k]=np.linspace(pos_arr[0,k],pos_arr[1,k],dt)
            
        for i in range(1,len(pos_arr)-1):
            for k in range(len(pos_arr[i,:])):
                new_pos_traj[i*dt-1:i*dt+dt,k]=np.linspace(pos_arr[i,k],pos_arr[i+1,k],dt+1)

        # quaternion interpolation:

        for j in range(t):
            for k in range(10):
                new_quat_traj[j*10+k,:]=self.lerp(quat_arr[j,:],quat_arr[j+1,:],float(k)/dt)
        
                
        new_obj_traj=PoseArray()
        new_obj_traj.header=obj_traj.header
        for i in range(len(new_pos_traj)):
            npos=Pose()
            npos.position.x=new_pos_traj[i,0]
            npos.position.y=new_pos_traj[i,1]
            npos.position.z=new_pos_traj[i,2]
            
            npos.orientation.x=new_quat_traj[i,0]
            npos.orientation.y=new_quat_traj[i,1]
            npos.orientation.z=new_quat_traj[i,2]
            npos.orientation.w=new_quat_traj[i,3]
            new_obj_traj.poses.append(npos)
        return new_obj_traj
    def get_traj_from_file(self,idx,folder=path_in_hand_data+"/trajectories/journal/open_loop/"):
        obj_=self.obj_list.arr[idx]
        j0=obj_.grasp
        obj_pose=obj_.initial_pose
        des_obj_pose=obj_.des
        fingers=obj_.fing
        
        print 'Object: '+obj_.name
        [j_traj,obj_traj]=pickle.load( open( folder+obj_.f_name+'.p', "rb" ) )
        return j_traj, obj_traj
        
    def rhc_traj(self, idx, T=10, OBJ_TRACKING=False, FROM_FILE=True,folder=path_in_hand_data+"/trajectories/journal/open_loop/"):
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
        
        
        for i in P_:
            i.send_signal(subprocess.signal.SIGINT)

        # Enable grav comp mode
        #raw_input("grav comp?")
        #self.allegro_drv.engage_grav_comp()
        
    def run_traj(self,idx, FROM_FILE=True, DBOT_TRACKING=True, OBJ_TRACKING=True,RECORD_DATA=False,RECORD_BAG_=False,trial_no=0,folder='',method='',df=[]):
        bag_folder='/media/bala/data_giskard/in_grasp_journal/interpolation_new'+'/'+method+'/'
        # Read idx from file
        obj_=self.obj_list.arr[idx]
        j0=obj_.grasp
        obj_pose=obj_.initial_pose
        des_obj_pose=obj_.des
        fingers=obj_.fing
        
        print 'Object: '+obj_.name
        # Enable grav comp mode
        #self.allegro_drv.engage_grav_comp()
        
        if(FROM_FILE):
            j_traj,obj_traj=self.get_traj_from_file(int(sys.argv[1]),folder=folder)
        else:
            # Get plan from in_grasp client:
            
            weights=self.weights
            f_j=j0.copy()+0.05
            f_j[8:12]=j0[8:12]
            start = time.time()
            j_traj, n_pose=self.in_grasp.call_server_list(j0, obj_pose, des_obj_pose, weights, fingers, T=2, final_joints=f_j)
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
            
            P_.append(self.tf_api.tf_pose_pub_process(obj_.aruco,'ar/obj','ar/marker') )

            raw_input('Placed Object and Aligned initial frames?')
            ar_initial_frame=self.tf_api.tf_find_transform('/i','ar/marker')
            P_.append(self.tf_api.tf_pose_pub_process(ar_initial_frame,'/ar/obj_pose','/ar/marker'))
            time.sleep(0.5)

        if(DBOT_TRACKING):
            initial_frame=self.tf_api.tf_find_transform('/i','dbot/obj')
            P_.append(self.tf_api.tf_pose_pub_process(initial_frame,'/dbot/obj_pose','dbot/obj'))
            
        if(RECORD_BAG_):
            bag_name=bag_folder+obj_.name+'_'+str(idx)+'_'+str(trial_no)
            bag_process=self.bag_api.record_bag_process(bag_name)
            
        # Run trajectory
        raw_input("Run trajectory?")
        self.allegro_drv.publish_joint_traj(j_traj,10)
        
        # Collect data
        if(RECORD_DATA):
            suc=raw_input('Press Enter - store final pose \n Press F to store as failure ')
            if(suc==''):
                # Get object pose from aruco marker
                ar_reached_pose=self.tf_api.tf_find_transform('ar/obj_pose')
                db_reached_pose=self.tf_api.tf_find_transform('dbot/obj_pose')
                # Write to panda data frame:
                df=self.pd_api.write_pd_data(df, [[method,obj_.name,idx,trial_no,obj_.des, ar_reached_pose, db_reached_pose]])
                print obj_.des
                print ar_reached_pose
                print db_reached_pose
                
            else:
                # Store failure
                obj_.reached='F'
            #objects.write_reached_pose(folder+trial_name)
        if(OBJ_TRACKING):
            for i in P_:
                i.send_signal(subprocess.signal.SIGINT)
        if(RECORD_BAG_):
            bag_process.send_signal(subprocess.signal.SIGINT)
        return df

    def run_mpc(self,idx,FROM_FILE=True, OBJ_TRACKING=True,AR_TRACKING=True, DBOT_TRACKING=True, RECORD_DATA=True, RECORD_BAG_=True, trial_no=0, folder='', method='', df=[],SIM_TRACKING=False):
        finish=False
        while(finish==False):
            bag_folder='/media/bala/data_giskard/in_grasp_journal/feedback/obj/'
            
            #in_grasp_pip=inGraspPipeline(init_node=True)
            #in_grasp_pip.read_objects()
            obj_=self.obj_list.arr[idx]
            j0=obj_.grasp
            
            if(FROM_FILE):
                j_traj,obj_traj=self.get_traj_from_file(idx,folder)
            else:
                j_traj,obj_traj,obj_pose=self.generate_traj(idx,WRITE_TO_FILE=False)
                
            # interpolate trajectories:
            j_traj=self.interpolate_jtraj(j_traj)
            obj_traj=self.interpolate_obj_traj(obj_traj)
            # move to initial pose:
            raw_input('Move to initial grasp?')
            for k in range(4):
                self.pub.publish(obj_traj)
            self.allegro_drv.publish_joints(j0)
            P_=[] # list of subprocesses
            if(SIM_TRACKING):
                # setup initial offsets:
                js_j0=JointState()
                js_j0.position=j0
                obj_pose=Pose()
                obj_pose.position.x=obj_.initial_pose[0]
                obj_pose.position.y=obj_.initial_pose[1]
                obj_pose.position.z=obj_.initial_pose[2]
                quaternion = tf.transformations.quaternion_from_euler(obj_.initial_pose[3], obj_.initial_pose[4],obj_.initial_pose[5])
                obj_pose.orientation.x=quaternion[0]
                obj_pose.orientation.y=quaternion[1]
                obj_pose.orientation.z=quaternion[2]
                obj_pose.orientation.w=quaternion[3]
                                
                self.in_grasp.setup_sim(js_j0,obj_pose,obj_.fing,3,10)
                #P_.append(self.tf_api.tf_pose_pub_process(obj_.offset,'/dbot/obj_pose','thumb_tip'))
            
            if(OBJ_TRACKING):
                
                raw_input('Placed Object?')
                P_.append(self.tf_api.tf_pose_pub_process(obj_.initial_pose,'i') )
                P_.append(self.tf_api.tf_pose_pub_process(obj_.offset,'rigid','thumb_tip'))
                P_.append(self.tf_api.tf_pose_pub_process(obj_.des,'G'))
        
                
            if(AR_TRACKING):
                P_.append(self.tf_api.tf_pose_pub_process(obj_.aruco,'ar/obj','ar/marker') )
                raw_input('Placed Object and Aligned initial frames?')
                ar_initial_frame=self.tf_api.tf_find_transform('/i','ar/marker')
                P_.append(self.tf_api.tf_pose_pub_process(ar_initial_frame,'/ar/obj_pose','/ar/marker'))
                time.sleep(0.1)

            if(DBOT_TRACKING):
                initial_frame=self.tf_api.tf_find_transform('/i','dbot/obj')
                P_.append(self.tf_api.tf_pose_pub_process(initial_frame,'/dbot/obj_pose','dbot/obj'))
            
            if(RECORD_BAG_):
                bag_name=bag_folder+obj_.name+'_'+str(idx)+'_'+str(trial_no)
                bag_process=self.bag_api.record_bag_process(bag_name)
            
            raw_input('Run mpc?')
            # call mpc service:
            mpc_server_name = rospy.get_param("/mpc_server_name")

            rospy.wait_for_service(mpc_server_name)
            run_mpc=rospy.ServiceProxy(mpc_server_name,RunMPC)
            result_=run_mpc(obj_traj,j_traj)
            # Collect data
            if(RECORD_DATA):
                suc=raw_input('Press Enter - store final pose \n Press F to store as failure ')
                if(suc==''):
                    # Get object pose from aruco marker
                    ar_reached_pose=self.tf_api.tf_find_transform('ar/obj_pose')
                    db_reached_pose=ar_reached_pose#self.tf_api.tf_find_transform('dbot/obj_pose')
                    print obj_.des
                    print ar_reached_pose
                    print np.linalg.norm(obj_.des[0:3]-ar_reached_pose[0:3])
                    run_again=raw_input('Press Enter - store final pose \n Press R to rerun ')
                    if(run_again==''):
                        # Write to panda data frame:
                        df=self.pd_api.write_pd_data(df, [[method+'-fb',obj_.name,idx,trial_no,obj_.des, ar_reached_pose, db_reached_pose]])
                        finish=True
                        #print db_reached_pose
                else:
                    # Store failure
                    obj_.reached='F'
                    #objects.write_reached_pose(folder+trial_name)
    
            if(len(P_)!=0):
                for i in P_:
                    i.send_signal(subprocess.signal.SIGINT)
            if(RECORD_BAG_):
                bag_process.send_signal(subprocess.signal.SIGINT)

                
        return df
def mpc_exp(n_trials=3):
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()
    method='ik_obj'
    folder=path_in_hand_data+'/interpolation/'+method+'/'
    pd_file=path_in_hand_data+'/waypoint_exp.csv'
    df=in_grasp_pip.pd_api.read_pd(pd_file)
    #columns=['method','Object','Trajectory','Trial','Desired Pose','AR Reached Pose','DBOT Reached Pose']
    #df=in_grasp_pip.pd_api.create_pd_frame(columns)

    for i in range(n_trials):
        df=in_grasp_pip.run_mpc(int(sys.argv[1]),OBJ_TRACKING=False,SIM_TRACKING=True,FROM_FILE=True,RECORD_DATA=False, DBOT_TRACKING=False,AR_TRACKING=False,RECORD_BAG_=False,trial_no=i,folder=folder,method=method,df=df)
    in_grasp_pip.pd_api.write_pd_file(pd_file,df)



def get_trajectories(folder,pd_file_name,method):
    in_grasp_pip=inGraspPipeline(init_node=True)
    file_name=path_in_hand_data+"/trajectories/waypoint_extra_goal_poses.csv"
    in_grasp_pip.read_objects(file_name)
    
    # create pd file
    df=in_grasp_pip.pd_api.read_pd(pd_file_name)
    #columns=['method','Object','Desired Pose','Reached Pose','Planning Time','RR cost','P_err','O_err']
    #df=in_grasp_pip.pd_api.create_pd_frame(columns)
    for i in range(0,len(in_grasp_pip.obj_list.arr)):
        print i
        des_pose,reached_pose,metrics=in_grasp_pip.generate_traj(i,folder=folder)
        df=in_grasp_pip.pd_api.write_pd_data(df,[[method,in_grasp_pip.obj_list.arr[i].name, des_pose,reached_pose, metrics[0], metrics[1], metrics[2], metrics[3]]])
        #print df
    # write to pandas file:
    in_grasp_pip.pd_api.write_pd_file(pd_file_name,df)
    
def waypoint_exp():
    methods=['obj','ik_obj','j','ik_j']
    i=1
    folder=path_in_hand_data+'/interpolation/'+methods[i]+'/'
    pd_file=path_in_hand_data+'/interpolation_new.csv'
    get_trajectories(folder,pd_file,methods[i])
    
def vis_traj(FROM_FILE=True):
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()
    i=2
    methods=['obj','ik_obj','j','ik_j']
    folder=path_in_hand_data+'/interpolation/'+methods[1]+'/'
    folder_rr=path_in_hand_data+'/interpolation/'+methods[2]+'/'
    if(FROM_FILE):
        j_traj,obj_traj=in_grasp_pip.get_traj_from_file(int(sys.argv[1]),folder=folder)
        _,obj_traj_rr=in_grasp_pip.get_traj_from_file(int(sys.argv[1]),folder=folder_rr)
    else:
        j_traj,obj_traj,obj_pose=in_grasp_pip.generate_traj(int(sys.argv[1]),WRITE_TO_FILE=False)
        

    r = rospy.Rate(1.0)
    i=0
    #print in_grasp_pip.obj_list.arr[int(sys.argv[1])].des
    des_pose=in_grasp_pip.pose_api.pose_from_arr(in_grasp_pip.obj_list.arr[int(sys.argv[1])].des)
    des_pose.header.frame_id='palm_link'
    while not rospy.is_shutdown() and i<2:
        # Publish desired object pose:
        in_grasp_pip.des_pub.publish(des_pose)
        in_grasp_pip.pub.publish(obj_traj)
        in_grasp_pip.pub_rr.publish(obj_traj_rr)
        #in_grasp_pip.traj_server.viz_joint_traj(j_traj,base_link='allegro_mount')
        #in_grasp_pip.j_pub.publish(j_traj)
        r.sleep()
        i+=1
    #in_grasp_pip.pose_api.plot_acc_profiles([obj_traj_rr,obj_traj],['min_acc','obj_interpol'])
    


def run_waypoint(n_trials=3):
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()

    methods=['obj','ik_obj','j','ik_j']
    k=2
    folder=path_in_hand_data+'/interpolation/'+methods[k]+'/'
    pd_file=path_in_hand_data+'/interpolation_new_exp.csv'
    df=in_grasp_pip.pd_api.read_pd(pd_file)
    #columns=['method','Object','Trajectory','Trial','Desired Pose','AR Reached Pose','DBOT Reached Pose']
    #df=in_grasp_pip.pd_api.create_pd_frame(columns)

    for i in range(n_trials):
        df=in_grasp_pip.run_traj(int(sys.argv[1]),OBJ_TRACKING=True,FROM_FILE=True,RECORD_DATA=True, DBOT_TRACKING=True,RECORD_BAG_=True,trial_no=i,folder=folder,method=methods[k],df=df)
    in_grasp_pip.pd_api.write_pd_file(pd_file,df)

def run_waypoint_test(n_trials=1):
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()

    methods=['obj','ik_obj','j','ik_j']
    k=2
    folder=path_in_hand_data+'/interpolation/'+methods[k]+'/'
    pd_file=path_in_hand_data+'/interpolation_exp.csv'
    df=in_grasp_pip.pd_api.read_pd(pd_file)
    #columns=['method','Object','Trajectory','Trial','Desired Pose','AR Reached Pose','DBOT Reached Pose']
    #df=in_grasp_pip.pd_api.create_pd_frame(columns)

    for i in range(n_trials):
        df=in_grasp_pip.run_traj(int(sys.argv[1]),OBJ_TRACKING=False,FROM_FILE=True,RECORD_DATA=False, DBOT_TRACKING=False,RECORD_BAG_=False,trial_no=i,folder=folder,method=methods[k],df=df)
    #in_grasp_pip.pd_api.write_pd_file(pd_file,df)

    
def run_open_loop(n_trials=5):
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()
    for i in range(n_trials):
        in_grasp_pip.run_traj(int(sys.argv[1]),OBJ_TRACKING=True,FROM_FILE=True,RECORD_DATA=True, DBOT_TRACKING=True,RECORD_BAG_=True,trial_no=i)

def run_open_loop_test():
    in_grasp_pip=inGraspPipeline(init_node=True)
    in_grasp_pip.read_objects()
    in_grasp_pip.run_traj(int(sys.argv[1]),OBJ_TRACKING=False,FROM_FILE=True,RECORD_DATA=False, DBOT_TRACKING=False,RECORD_BAG_=False)

    
if __name__=='__main__':
    #idx=int(sys.argv[1])
    #in_grasp_pip=inGraspPipeline(init_node=True)
    #in_grasp_pip.read_objects()
    #get_trajectories()
    #vis_traj(False)
    #run_mpc(FROM_FILE=True)
    #run_open_loop()
    #run_open_loop_test()
    #in_grasp_pip=inGraspPipeline(init_node=True,publish_prefix='/allegroHand_0',listen_prefix='/allegroHand_0')
    #waypoint_exp()
    #vis_traj()
    #run_waypoint()
    #run_waypoint_test()
    mpc_exp()
