import rospy
import subprocess
from subprocess import Popen
class rosbagAPI:
    def __init__(self):
        print 'initialized rosbag api'

    def record_bag_process(self, file_name, topics=['/tf', '/camera/rgb/image_raw', '/camera/rgb/camera_info', '/camera/depth/camera_info','/camera/depth/image_raw','/cv_camera/image_raw','/cv_camera/camera_info', '/allegro_hand_right/joint_states', '/allegro_hand_right/joint_cmd']):
        cmd=['/bin/bash','rosrun','rosbag','record','-O',file_name]+topics
        proc=Popen(cmd, stdout=subprocess.PIPE)
        return proc


