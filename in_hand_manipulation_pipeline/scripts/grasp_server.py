#!/usr/bin/env python
# This server sends joint positions to the allegro hand for a preshape.
# Input:
# Output:
import rospy
from in_hand_manipulation_pipeline.srv import *
import sys
import time
import numpy as np
from rospkg import RosPack


# Import robot hand api
rp=RosPack()
rp.list()
path=rp.get_path('allegro_driver')+'/scripts'
sys.path.insert(0,path)
from allegro_ros_api import handAPI


def send_grasp(req):
    allegro=handAPI()
    j_cmd=req.hand_preshape
    allegro.publish_preshape(j_cmd.position)
    return True

def grasp_vel_servo(req):
    allegro=handAPI()
    allegro.loop_rate.sleep()
    joints=list(req.joints)
    time.sleep(0.1)
    grasped=allegro.vel_servo(req.thresh.data,req.delta_vel.data,joints)
    return grasped

def delta_q_move(req):
    allegro=handAPI()
    allegro.loop_rate.sleep()
    joints=list(req.joints)
    time.sleep(0.1)
    status_=allegro.move_delta_q(req.delta_q.data,joints)
    return status_
    
if __name__=="__main__":
    
    rospy.init_node("grasp_server")

    # Grasp preshape server
    s=rospy.Service('grasp/send_preshape',SendPreshape,send_grasp)


    # Velcity servoing server
    vel_s=rospy.Service('grasp/grasp_vel_servo',GraspVelServo,grasp_vel_servo)

    # Move joints by delta_q
    del_q_srv=rospy.Service('grasp/move_delta_q_joints',DeltaqJoints,delta_q_move)
    rospy.spin()
