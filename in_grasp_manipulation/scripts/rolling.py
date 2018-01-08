
import sys
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from rolling_client import rollingClient, fgClient
from rospkg import RosPack
import numpy as np
import copy
rp=RosPack()
path_allegro=rp.get_path('allegro_driver')+'/scripts'
sys.path.insert(0,path_allegro)
from allegro_ros_api import handAPI
path_=rp.get_path('optimization_pkg')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *
path_in_grasp=rp.get_path('in_grasp_manipulation')+'/python'
sys.path.insert(0,path_in_grasp)
import kdl_allegro_model

class roll:
    def __init__(self):
        self.allegro_kdl=kdl_allegro_model.allegroKinematics()
        publish_prefix='/allegro_hand_right'
        listen_prefix=publish_prefix
        self.allegro_drv=handAPI(init_node=True,publish_prefix=publish_prefix,listen_prefix=listen_prefix)
        self.traj_server=trajectoryServer(10,init_node=False,topic_name='/in_grasp/jtraj',robot_name='allegro')
         
        
        # roll object
        # initial object pose
        # desired object rotation
        self.roll_client=rollingClient()
        self.fg_client=fgClient()
        
    def roll_obj(self,j_initial):
        js=JointState()
        js.position=j_initial
        roll_client=rollingClient()
        fg_client=fgClient()
        # get pose of object by computing centroid from fingertips:
        o_mat=self.allegro_kdl.min_object_pose(j_initial,[0,1,2,3])
        f_mat=self.allegro_kdl.min_object_pose(j_initial,[0,1,2])
        
        #print o_mat
        # initial obj_pose:
        obj_pose=Pose()
        obj_pose.position.x=f_mat[0]
        obj_pose.position.y=o_mat[1]
        obj_pose.position.z=o_mat[2]
        
        obj_pose.orientation.x=0.0
        obj_pose.orientation.y=0.0
        obj_pose.orientation.z=0.0
        obj_pose.orientation.w=1.0
        # desired obj_pose:
        des_obj_pose=Pose()
        des_obj_pose.position.x=f_mat[0]-0.01
        des_obj_pose.position.y=o_mat[1]
        des_obj_pose.position.z=o_mat[2]-0.01
        des_obj_pose.orientation.x=0.0
        des_obj_pose.orientation.y=-0.259
        des_obj_pose.orientation.z=0.0
        des_obj_pose.orientation.w= 0.966
    
        j_traj=self.roll_client.call_server(js,obj_pose,des_obj_pose)

        # visualize jtraj:
        self.traj_server.viz_joint_traj(j_traj,base_link='palm_link')

        raw_input('send traj?')
        # run trajectory on robot
        self.allegro_drv.publish_joint_traj(j_traj,10)
        return j_traj.points[-1].positions

    def fg(self,j_init):
        j_init=np.array(j_init)
        init_js=JointState()
        init_js.position=copy.deepcopy(j_init)
        des_js=JointState()
        des_js.position=copy.deepcopy(j_init)
        fingers=[0,1,2]
        for i in fingers:
            #raw_input('next finger?')
            init_js.position=copy.deepcopy(des_js.position)
            # remove finger:
            des_js.position[i*4+2]=j_init[i*4+2]-0.5
            des_js.position[i*4+3]=j_init[i*4+3]-0.2
            self.fg_client.call_server(i,init_js,des_js,0.05)
            des_js.position[i*4+1]=j_init[i*4+1]+0.4
            self.fg_client.call_server(i,init_js,des_js,0.05)
        return des_js.position
    def envelope(self,j_init):
        j_init=np.array(j_init)
        init_js=JointState()
        init_js.position=copy.deepcopy(j_init)
        des_js=JointState()
        des_js.position=copy.deepcopy(j_init)
        fingers=[0,1,2]
        for i in fingers:
            raw_input('next finger?')
            init_js.position=copy.deepcopy(des_js.position)
            # remove finger:
            des_js.position[i*4+1]=j_init[i*4+1]+0.6
            des_js.position[i*4+2]=j_init[i*4+2]+0.6
            des_js.position[i*4+3]=j_init[i*4+3]+0.5
            self.fg_client.call_server(i,init_js,des_js,0.05)
           
        return des_js.position
if __name__=='__main__':
    r=roll()
     # move to initial grasp
    j_initial=[0.2,0.8,0.9,0.5,
               0.0,0.8,0.9,0.5,
               0.0,0.8,0.9,0.5,
               1.65,0.0,0.7,0.1]

    raw_input('initial grasp?')
    r.allegro_drv.publish_joints(j_initial)
    
    for i in range(3):
    # roll object:
        in_grasp_position=r.roll_obj(j_initial)
        raw_input('fg?')
        # finger gaiting to repose fingers on object:
        j_initial=r.fg(in_grasp_position)
    # envelop object:
    r.envelope(j_initial)

