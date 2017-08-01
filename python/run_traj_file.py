# Runs the trajectory on the robot
import robot_node as robot
from object_list import *
from subprocess import Popen
import subprocess
import os
import signal
import time
import sys
n_trials=5
MAKE_TRIAL_LIST=False
ARUCO_=False
RECORD_VIDEO_=False

#EXP_NAME='exp_1_waypoint'
def read_objects(file,folder):
    obj_list=TrajList(file,folder)
    return obj_list


if __name__=='__main__':
    f_name='../data/trajectories/debug/Apple_0_0'

    allegroStates=robot.robotStates(listen_aruco=ARUCO_)   
    # Publishing joint positions to robot
    allegroStates.position_mode()
    #allegroStates.publish_grasp()
    
    
    # Run trajectory       
    joint_traj=np.loadtxt(open(f_name,'r'))
        
    raw_input('run_traj')
    allegroStates.publish_joint_trajectory(joint_traj)
        
        

