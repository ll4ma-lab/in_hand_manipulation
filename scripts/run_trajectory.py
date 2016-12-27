# Runs the trajectory on the robot
import robot_node as robot
from object_list import *
import rosbag
from std_msgs.msg import Int32, String

n_trials=1
MAKE_TRIAL_LIST=true

def read_objects(file,folder):
    obj_list=TrajList(file,folder)
    return obj_list


if __name__=='__main__':
    allegroStates=robot.robotStates()
    f_name
    trial_name
    folder

    if(MAKE_TRIAL_LIST):
        ''' RUN only once'''
        # read file and store as object list
        obj_=read_objects('../data/goal_poses/waypoint_goal_poses',' ')=  
        # Make trial list from object list
        obj_.write_trial_list(n_trials,trial_name)
        
    '''
    # Read trial list
    objects=TrialList(trial_name,folder,n_trials)


    repeat=1
    
    while(repeat<2):
        if(repeat==1):
            t_no=int(input('Ready to move to initial grasp, enter trajectory number: '))
            # Get trajectory number
            print 'Trajectory number: '+str(t_no)
            print 'Object: '+objects.arr[t_no].name
        trial_no=int(input('Enter trial number: '))

        # Publishing joint positions to robot
        allegroStates.position_mode()
        allegroStates.publish_grasp(objects.arr[t_no].grasp)
    
        raw_input('Moved to initial grasp, place object and press enter to run trajectory ')
        # Record bag file

        #Run trajectory       
        suc=int(raw_input('Finished running trajectory,\n Press 1 - store final pose \n Press 2 - to store as failure '))
        if(suc==1):
            # Store final object pose
            objects.arr[t_no].sol=joint_angles
        else:
            # Store failure
            objects.arr[t_no].sol='F'

        objects.write_reached_pose('../data/goal_poses/waypoint_goal_joint_angles.csv')
    
        repeat=int(input('Press 0- to run another trial \n Press 1-to run different trajectory.\n Press 2- to exit program\n '))
    '''
    


