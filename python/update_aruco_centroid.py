# Stores new grasps for objects
import robot_node as robot
import sys
import csv
import tf 
import numpy as np
from object_list import *
import rospy




def main():
    # Get offset from database relating aruco marker to object centroid
    file="../data/object_grasps/initial_centroids_way.csv"
    folder_to_write=""
    obj_list=ObjectReachList(file,folder_to_write)  
    
    allegro=robot.robotStates(listen_aruco=True)           
    got_aruco_pose=False
    repeat=1

    while(repeat==1):
        obj_no=int(input('enter object'))
        print obj_list.arr[obj_no].name
        allegro.publish_grasp(obj_list.arr[obj_no].grasp)
        raw_input('place object')
        
        # Get Aruco pose
        
        (trans1,rot1)= allegro.listener.lookupTransform('/palm_link', '/ar', rospy.Time(0))
        
        
        # Write offset to file
        euler=tf.transformations.euler_from_quaternion(rot1)
        tx=euler[0]
        ty=euler[1]
        tz=euler[2]
        obj_list.arr[obj_no].offset=[tx,ty,tz]
        got_aruco_pose=True
        
        repeat=int(input('repeat(1):'))
        
        
    with open("../data/object_grasps/centroid.csv",'w') as f:
        writer=csv.writer(f)
        for i in range(len(obj_list.arr)):
            list_data=[obj_list.arr[i].name,','.join(map(str,obj_list.arr[i].offset)),obj_list.arr[i].grasp]
            writer.writerow(list_data)

            break





if __name__=='__main__':
    main()






