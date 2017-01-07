# Reads previusly generated goal poses file and update goal pose
import robot_node as robot
from object_list import *


def read_objects(file,folder):
    obj_list=ObjectReachList(file,folder)
    return obj_list

if __name__=='__main__':
    allegroStates=robot.robotStates()


    # read file and store as object list
    objects=read_objects('../data/goal_poses/feas_goal_joint_angles.csv',' ')
    repeat=1
    update_objects=[]
    while(repeat==1):
        t_no=int(input('Ready to move to desired grasp, enter trajectory number: '))
        # Get trajectory number
        print 'Trajectory number: '+str(t_no)
        print 'Object: '+objects.arr[t_no].name
        # Publishing joint positions to robot
        allegroStates.position_mode()
        allegroStates.publish_grasp(objects.arr[t_no].des)
    
        raw_input('Moved to desired grasp, press enter to activate grav. comp. mode ')
        # Switching to gravity compensation
        allegroStates.grav_comp_mode()
        raw_input('Switched to gravity compensation mode, correct desired pose and press enter: ')

    

        # Store joint positions
        allegroStates.loop_rate.sleep()
        joint_angles=allegroStates.joint_states.position
        objects.arr[t_no].sol=joint_angles
        update_objects.append(objects.arr[t_no])
        repeat=int(input(' Press 1-to get more goal poses.\n Press 2- to write to file and exit.\n '))
    
    
    objects.write_sol_from_tjo('../data/goal_poses/goal_joint_angles.csv')
    objects.arr=update_objects
    objects.write_sol_from_tjo('../data/goal_poses/new_updated_goal_joint_angles.csv')

