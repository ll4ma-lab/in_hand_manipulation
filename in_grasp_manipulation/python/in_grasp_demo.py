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
    method='relaxed-rigidity'
    f_name='/home/bala/catkin_ws/src/in_hand_manipulation/in_hand_manipulation_data/data/in_grasp_manipulation/trajectories/debug_'+method+'_solution.csv'
    trial_name=method+'_deb.csv'
    folder='/home/bala/catkin_ws/src/in_hand_manipulation/in_hand_manipulation_data/data/in_grasp_manipulation/trials/'
    #traj_folder='../data/trajectories/reachable_poses/'+method+'/'
    traj_folder='/home/bala/catkin_ws/src/in_hand_manipulation/in_hand_manipulation_data/data/in_grasp_manipulation/trajectories/debug/'
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
    #objects.add_aruco_centroid_locations('../data/object_grasps/object_centroids_4f.csv')

    repeat=1
    t_no=int(sys.argv[1])
    print t_no
    trial_no=-1
    while(repeat<2):
        print 'Object: '+objects.arr[t_no*n_trials].name
        print 'Object: '+objects.arr[t_no*n_trials].f_name

        trial_no=0
        index=t_no*n_trials
        raw_input('send grasp?')
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
            if(RECORD_VIDEO_):
                v_name=method+'_'+objects.arr[index].name+'_'+str(t_no)
                # script to record video
                python_args=['python','../../object_pose_vis/scripts/obj_pose_vis.py',v_name]
                vid_process=allegroStates.run_process(python_args)
                time.sleep(1.0)
                
        else:
            raw_input('Placed Object and Aligned initial frames?')

        # Run trajectory       
        joint_traj=np.loadtxt(open(traj_folder+objects.arr[index].f_name,'r'))
        

        allegroStates.publish_joint_trajectory(joint_traj)
        time.sleep(1.0)
        if(RECORD_VIDEO_):
            vid_process.send_signal(subprocess.signal.SIGINT)
            time.sleep(0.1)
        if(ARUCO_):
           for i in P_:
                i.send_signal(subprocess.signal.SIGINT)
                time.sleep(0.1)
        
        next_=raw_input('Press Enter- to run different trajectory.\nPress 2- to exit program\n ')
        if(next_==''):
            repeat=1
        else:
            repeat=int(next_)
        
        
        

