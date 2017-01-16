import sys
import rosbag
import tf
import subprocess
import time
import robot_node as robot
from object_list import *
import os  
class BagProcessing:
    def __init__(self,file_name):
        self.bag=rosbag.Bag(file_name)
        self.contents = self.bag.read_messages()
        
        topics = []
	for topic, msg, t in self.contents:
		if topic not in topics:
                    topics.append(topic)
        self.topics=topics
        print topics
    def getTF(self):
        for subtopic, msg, t in self.bag.read_messages('/tf'):
            for i in range(len(msg.transforms)):
                tf_data=msg.transforms[i]
                if tf_data.child_frame_id=='/obj':
                    print msg



def play_bag(file_name):
    cmd=['/bin/bash','rosrun','rosbag','play',file_name]
    proc=subprocess.Popen(cmd,stdout=subprocess.PIPE)
    return proc



if __name__=='__main__':
    allegroStates=robot.robotStates(listen_aruco=True)
    method='relaxed-full-linear'
    trial_name='random_'+method+'.csv'
    folder='../data/trials/'
    bag_folder='/media/bala/Experiments_urlg/icra_2016_in_grasp/rss-experiments-bag/random/'+method+'/'
    objects=TrialList(trial_name,folder,5)
    bag_files=0
    t_no=10
    while t_no<40:
        for trial_no in range(5):
            index=t_no*5+trial_no
            bag_name=bag_folder+objects.arr[index].name+'_'+str(t_no)+'_'+str(trial_no)+'.bag'
            if os.path.isfile(bag_name):
               bag_files+=1
               p=play_bag(bag_name)
               stdoutdata, stderrdata = p.communicate()
               reached_pose=allegroStates.tf_find_transform('obj')
               objects.arr[index].reached=reached_pose
            else:
                objects.arr[index].reached='F'
            objects.write_reached_pose(folder+trial_name)
            print t_no
        t_no+=1
