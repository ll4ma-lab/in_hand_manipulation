# Stores a goal pose based on the thumb offset to a file
import robot_node as robot
from object_list import *


def read_objects(file,folder):
    obj_list=ObjectReachList(file,folder)
    return obj_list

if __name__=='__main__':
    allegroStates=robot.robotStates()


    # read file and store as object list
    objects=read_objects('../data/object_grasps/waypoint_initial_poses.csv',' ')
    repeat=1
    
    while(repeat==1):
        t_no=int(input('Ready to move to initial grasp, enter trajectory number: '))
        # Get trajectory number
        print 'Trajectory number: '+str(t_no)
        print 'Object: '+objects.arr[t_no].name
        # Publishing joint positions to robot
        allegroStates.position_mode()
        allegroStates.publish_joint_positions(objects.arr[t_no].grasp)
    
        raw_input('Moved to initial grasp, press enter to activate grav. comp. mode ')
        # Switching to gravity compensation
        allegroStates.grav_comp_mode()
        raw_input('Switched to gravity compensation mode, move to desired pose and press enter: ')

    

        # Store joint positions
        allegroStates.loop_rate.sleep()
        joint_angles=allegroStates.joint_states.position
        objects.arr[t_no].sol=joint_angles
        repeat=int(input(' Press 1-to get more goal poses.\n Press 2- to write to file and exit.\n '))
    
    
    objects.write_sol_from_tjo('../data/goal_poses/waypoint_goal_joint_angles.csv')


