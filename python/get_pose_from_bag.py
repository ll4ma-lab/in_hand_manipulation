import sys
import rosbag
import tf
import subprocess
import time
import robot_node as robot
from object_list import *
import os  
import PyKDL
import rospy
from rosgraph_msgs.msg import Clock
from multiprocessing import Pool

class BagProcessing:
    def __init__(self,file_name):
        if os.path.isfile(bag_name):

            self.bag=rosbag.Bag(file_name)
            self.contents = self.bag.read_messages()
        
            topics = []
            for topic, msg, t in self.contents:
		if topic not in topics:
                    topics.append(topic)
            self.topics=topics
        else:
            exit()
            
    def getTF(self):
        for subtopic, msg, t in self.bag.read_messages('/tf'):
            for i in range(0,len(msg.transforms)):
                tf_data=msg.transforms[i]
                if tf_data.child_frame_id=='/obj' and tf_data.header.frame_id=='/obj_ar':
                    t_pose=np.zeros(7)
                    t_pose[0]=tf_data.transform.translation.x
                    t_pose[1]=tf_data.transform.translation.y
                    t_pose[2]=tf_data.transform.translation.z
                    t_pose[3]=tf_data.transform.rotation.x
                    t_pose[4]=tf_data.transform.rotation.y
                    t_pose[5]=tf_data.transform.rotation.z
                    t_pose[6]=tf_data.transform.rotation.w

      
        return t_pose


def play_bag(file_name):
    cmd=['/bin/bash','rosrun','rosbag','play','-u','2','--clock',file_name,]
    proc=subprocess.Popen(cmd,stdout=subprocess.PIPE)
    return proc


def transform_pose(t_pose,i_pose):
    # transforms i_pose to t_pose frame:
    # generate T mat from t_pose
    T=np.eye(4)
    T[0:3,3]=t_pose[0:3]
    R=PyKDL.Rotation()
    R=R.Quaternion(t_pose[3],t_pose[4],t_pose[5],t_pose[6])

    for k in range(3):
        for l in range(3):
            T[k,l]=R[k,l]

    T=np.linalg.inv(T)
    T_i=np.eye(4)
    T_i[0:3,3]=i_pose[0:3]
    R=PyKDL.Rotation()
    R=R.RPY(i_pose[3],i_pose[4],i_pose[5])
    for k in range(3):
        for l in range(3):
            T_i[k,l]=R[k,l]

    obj_T=np.matrix(T_i)*np.matrix(T)

    R=PyKDL.Rotation(obj_T[0,0],obj_T[0,1],obj_T[0,2],obj_T[1,0],obj_T[1,1],obj_T[1,2],obj_T[2,0],obj_T[2,1],obj_T[2,2])

    new_pose=np.zeros(6)
    new_pose[0:3]=obj_T[0:3,3].T
    new_pose[3:6]=np.array(R.GetRPY())
    return new_pose

def run_all():
    method='relaxed-full-linear'
    trial_name=method+'_init.csv'
    folder='../data/trials/'
    bag_folder='/media/bala/Experiments_urlg/icra_2016_in_grasp/rss-experiments-bag/exp2/'+method+'/'
    objects=TrialList(trial_name,folder,5)
    bag_files=0
    t_no=66
    allegroStates=robot.robotStates(listen_aruco=True,clock_bag=True)
    while t_no<76:
        for trial_no in range(5):
            index=t_no*5+trial_no
            bag_name=bag_folder+objects.arr[index].name+'_'+str(t_no)+'_'+str(trial_no)+'.bag'
            #print bag_name
            if os.path.isfile(bag_name):
               allegroStates.reinitialize_tf()
               bag_files+=1
               p=play_bag(bag_name)
               #a=rospy.Subscriber('/clock',Clock,allegroStates.clock_callback)
               stdoutdata, stderrdata = p.communicate()

            
               init_pose_transform=allegroStates.tf_find_transform('obj_ar','obj')
               # Transform 
               initial_pose=transform_pose(init_pose_transform,objects.arr[index].initial_pose)
               objects.arr[index].exp_pose=initial_pose
               #print list(objects.arr[index].length)
            else:
                objects.arr[index].exp_pose='F'
            objects.write_exp_pose(folder+method+'_init_1.csv')
            # kill node:
            #rospy.signal_shutdown('test') 
            print t_no
        t_no+=1

def bag_multi(data):
    bag_name=data[0]
    initial=data[1]
    index=data[2]
    if os.path.isfile(bag_name):
        bag_=BagProcessing(bag_name)
        init_pose_transform=bag_.getTF()
        initial_pose=transform_pose(init_pose_transform,initial)
    else:
        initial_pose='F'
    exp_pose=initial_pose
    return [exp_pose,index]

if __name__=='__main__':
    method='trinkle'
    trial_name=method+'.csv'
    folder='../data/trials/'
    bag_folder='/home/bala/'+method+'/'
    objects=TrialList(trial_name,folder,5)

    bag_files=0
    t_no=0
    data_arr=[]
    while t_no<100:
        for trial_no in range(5):
            index=t_no*5+trial_no
            bag_name=bag_folder+objects.arr[index].name+'_'+str(t_no)+'_'+str(trial_no)+'.bag'       
            data_arr.append([bag_name,objects.arr[index].initial_pose,index])
        t_no+=1

    tr=0
    sc_pool=Pool()
    for res in sc_pool.imap_unordered(bag_multi,data_arr,chunksize=1):
        row=res
        q=row[-1]
        objects.arr[q].exp_pose=row[0]
        tr+=1
        print tr

        
    objects.write_exp_pose(folder+method+'_init.csv')    
