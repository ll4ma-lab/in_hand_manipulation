# Runs the trajectory on the robot
import robot_node as robot
from object_list import *
from subprocess import Popen

n_trials=5
MAKE_TRIAL_LIST=False
ARUCO_=False
RECORD_BAG_=False

EXP_NAME='exp_1_waypoint'
def read_objects(file,folder):
    obj_list=TrajList(file,folder)
    return obj_list


if __name__=='__main__':
    allegroStates=robot.robotStates(listen_aruco=ARUCO_)
    f_name='../data/trajectories/waypoint_solution_exp.csv'
    trial_name='waypoint_5.csv'
    folder='../data/trials/'
    traj_folder='../data/trajectories/waypoint/'

    bag_folder='/media/bala/Experiments_urlg/icra_2016_in_grasp/rss-experiments-bag/waypoint-exp1'
    if(MAKE_TRIAL_LIST):
        ''' RUN only once'''
        # read file and store as object list
        obj_=TrajList(f_name,folder)
        # Make trial list from object list
        obj_.write_trial_list(n_trials,trial_name)

    # Read trial list
    objects=TrialList(trial_name,folder,n_trials)


    repeat=1
    t_no=0
    trial_no=0
    while(repeat<2):
        if(repeat==1):
            t_no_input=raw_input('Trajectory number ('+str(t_no)+'): ')
            if(t_no_input!=''):
                t_no=int(t_no_input)
            else:
                t_no+=1
            # Get Object
            print 'Object: '+objects.arr[t_no].name
            trial_no=0
        trial_no_input=raw_input('Trial number('+str(trial_no)+'): ')
        if(trial_no_input!=''):
            trial_no=int(trial_no_input)
        else:
            trial_no+=1
        index=t_no+trial_no

        # Publishing joint positions to robot
        allegroStates.position_mode()
        allegroStates.publish_grasp(objects.arr[index].grasp)
        raw_input('Placed object?')
        if(ARUCO_):
            allegroStates.tf_pose_pub(objects.arr[index].initial_pose,'i') 
            allegroStates.tf_pose_pub(objects.arr[index].offset,'rigid')
            allegroStates.tf_pose_pub(objects.arr[index].des,'G')
            raw_input('Aligned initial frames?')

            # Setup initial frame
            initial_frame=allegroStates.tf_find_transform('/ar','/i')
            allegroStates.tf_pose_pub(initial_frame,'/obj','/ar')          
        
            raw_input('Initial Frame obtained, Press enter to run trajectory')

            if(RECORD_BAG_):
                # recording bag file
                bag_name=bag_folder+objects.arr[index].name+'_'+str(index)
                
                bag_process=Popen(["/bin/bash","bag_record.sh",bag_name])
                sleep(2)

        # Run trajectory       
        joint_traj=np.loadtxt(open(traj_folder+objects.arr[index].f_name,'r'))
        

        allegroStates.publish_joint_trajectory(joint_traj)
        if(ARUCO_):
            suc=int(raw_input('Press 1 - store final pose \n Press 2 - to store as failure '))
            reached_pose=allegroStates.tf_find_transform('obj')
            if(RECORD_BAG_):
                bag_process.terminate()
            if(suc==1):
                # Get object pose from aruco marker
                
                # Get final joint angles
                joint_angles=allegroStates.joint_states.position
                # Get final object pose
                objects.arr[t_no].reached=[[joint_angles],reached_pose]
            else:
                # Store failure
                objects.arr[t_no].reached='F'
        
                objects.write_reached_pose(folder+trial_name)
    
        repeat=int(input('Press 0- to run another trial \nPress 1-to run different trajectory.\nPress 2- to exit program\n '))

