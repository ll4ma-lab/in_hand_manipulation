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
#MAKE_TRIAL_LIST=False
MAKE_TRIAL_LIST=True
ARUCO_=True
RECORD_BAG_=True

#EXP_NAME='exp_1_waypoint'
def read_objects(file,folder):
    obj_list=TrajList(file,folder)
    return obj_list


if __name__=='__main__':
    method=sys.argv[1]
    f_name='../data/trajectories/general_'+method+'_solution.csv'
    trial_name=method+'_general.csv'
    folder='../data/trials/'
    traj_folder='../data/trajectories/general/'+method+'/'
    
    bag_folder='/media/bala/Experiments_urlg/icra_2016_in_grasp/rss-experiments-bag/general/'+method+'/'
    if(MAKE_TRIAL_LIST):
        ''' RUN only once'''
        # read file and store as object list
        obj_=TrajList(f_name,folder)
        # Make trial list from object list
        obj_.write_trial_list(n_trials,trial_name)
        exit()
   
    allegroStates=robot.robotStates(listen_aruco=ARUCO_)   
    # Read trial list
    objects=TrialList(trial_name,folder,n_trials)
    objects.add_aruco_centroid_locations('../data/object_grasps/object_centroids_4f.csv')

    repeat=1
    t_no=-1
    trial_no=-1
    while(repeat<2):
        if(repeat==1):
            t_no_input=raw_input('Trajectory number ('+str(t_no+1)+'): ')
            if(t_no_input!=''):
                t_no=int(t_no_input)
            else:
                t_no+=1
            # Get Object
            print 'Object: '+objects.arr[t_no*n_trials].name
            trial_no=-1
        trial_no_input=raw_input('Trial number('+str(trial_no+1)+'): ')
        if(trial_no_input!=''):
            trial_no=int(trial_no_input)
        else:
            trial_no+=1
        index=t_no*n_trials+trial_no

         # Publishing joint positions to robot
        allegroStates.position_mode()
        allegroStates.publish_grasp(objects.arr[index].grasp)
    
        if(ARUCO_):

            P_=[] # list of subprocesses
            P_.append(allegroStates.tf_pose_pub_process(objects.arr[index].initial_pose,'i') )
            P_.append(allegroStates.tf_pose_pub_process(objects.arr[index].offset,'rigid','thumb_tip'))
            P_.append(allegroStates.tf_pose_pub_process(objects.arr[index].des,'G'))
            
            P_.append(allegroStates.tf_pose_pub_process(objects.arr[index].aruco,'obj_ar','ar') )

            raw_input('Placed Object and Aligned initial frames?')
            allegroStates.loop_rate.sleep()
            # Setup initial frame
            initial_frame=allegroStates.tf_find_transform('/i','/obj_ar')
            #palm_rotation=allegroStates.tf_find_transform('/palm_link','/obj_ar')
            P_.append(allegroStates.tf_pose_pub_process(initial_frame,'/obj','/obj_ar'))
            time.sleep(0.5)

            
            if(RECORD_BAG_):
                # recording bag file
                bag_name=bag_folder+objects.arr[index].name+'_'+str(t_no)+'_'+str(trial_no)
                bag_process=allegroStates.record_bag_process(bag_name)
                time.sleep(1)
        else:
            raw_input('Placed Object and Aligned initial frames?')

        # Run trajectory       
        joint_traj=np.loadtxt(open(traj_folder+objects.arr[index].f_name,'r'))
        

        allegroStates.publish_joint_trajectory(joint_traj)
        if(ARUCO_):
            suc=raw_input('Press Enter - store final pose \n Press F to store as failure ')

            if(suc==''):
                # Get object pose from aruco marker
                
                # Get final joint angles
                #joint_angles=allegroStates.joint_states.position
                # Get final object pose
                reached_pose=allegroStates.tf_find_transform('obj')
                objects.arr[index].reached=reached_pose
                print reached_pose[0:3]
            else:
                # Store failure
                objects.arr[index].reached='F'
            if(RECORD_BAG_):
                bag_process.send_signal(subprocess.signal.SIGINT)
                time.sleep(0.5)
            objects.write_reached_pose(folder+trial_name)

            for i in P_:
                i.send_signal(subprocess.signal.SIGINT)
            
        next_=raw_input('Press Enter- to run another trial \nPress 1-to run different trajectory.\nPress 2- to exit program\n ')
        if(next_==''):
            repeat=0
        else:
            repeat=int(next_)
        
        
        

