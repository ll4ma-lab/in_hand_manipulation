# Stores new grasps for objects
from handStates import *
import sys
import csv
import tf 
import numpy as np
class Object:
    def __init__(self,name):
        self.name=name

if __name__=='__main__':
    # Read csv file containing aruco marker offsets
    f_name='new_grasps.csv'
    obj_list=[]
    lines=[]
    with open(f_name,'r') as csvfile:
        csvreader=csv.reader(csvfile,delimiter=',',quotechar='"')
        for row in csvreader:
            lines.append(row)
            if(csvreader.line_num>1):
                object=Object(row[0])
                object.fing=row[1]
                object.aruco=np.array(map(float, row[2].split(',')))
                object.aruco_st=row[2]
                object.joints=row[3]
                object.offset=row[4]
                obj_list.append(object)



    # Get offset from database relating aruco marker to object centroid
      
        
    allegro=handStates()
    listener=tf.TransformListener()
    got_aruco_pose=False
    got_thumb_pose=False
    obj_no=int(sys.argv[1])
    print obj_list[obj_no].name
    
    while not rospy.is_shutdown():       
        # Read joint angles of hand
        joints=allegro.curr_joints_position

        # Get Aruco pose
        try:
            (trans1,rot1)= listener.lookupTransform('/palm_link', '/ar', rospy.Time(0))
            got_aruco_pose=True
        except: 
            continue

        # Add offset to Aruco pose and publish as object pose
        x=obj_list[obj_no].aruco[0]
        y=obj_list[obj_no].aruco[1]
        z=obj_list[obj_no].aruco[2]
        obj_pose=tf.TransformBroadcaster()
        obj_pose.sendTransform((x,y,z),(0.0,0.0,0.0,1.0),rospy.Time.now(),'/obj','/ar')
        

        # Get offset between thumb pose and object pose
        try:
            (trans,rot)= listener.lookupTransform('/thumb_tip', '/obj', rospy.Time(0))
            got_thumb_pose=True
            offset=trans
        except: 
            continue

        if(got_thumb_pose):
            # Write offset to file
            obj_list[obj_no].offset=offset
            with open("new_grasps.csv",'w') as f:
                writer=csv.writer(f)
                i=0
                for row in lines:
                    if(i>0):
                        if(obj_no+1==i):
                            list_data=[obj_list[i-1].name,obj_list[i-1].fing,obj_list[i-1].aruco_st,obj_list[i-1].joints,','.join(map(str,obj_list[i-1].offset))]
                        else:
                            list_data=[obj_list[i-1].name,obj_list[i-1].fing,obj_list[i-1].aruco_st,obj_list[i-1].joints,obj_list[i-1].offset]
                        data=list_data
                    else:
                        data=list(row)
                    writer.writerow(data)
                    i+=1

            break
        
        allegro.rate.sleep()
    












