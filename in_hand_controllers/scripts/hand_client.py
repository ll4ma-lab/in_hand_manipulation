#!/usr/bin/env python

# Copyright (C) 2018  Balakumar Sundaralingam, LL4MA Lab, University of Utah

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import numpy as np
import copy

from hand_services.srv import SendHandCommand
from sensor_msgs.msg import JointState
from trajectory_smoothing.srv import GetSmoothTraj
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class handClient:
    def __init__(self,hand_srv='allegro/hand_service',init_node=False, node_name='allegro_hand_client',listen_prefix='allegro_hand_right'):
        if(init_node):
            rospy.init_node(node_name)
            

        self.srv_name=hand_srv
        self.joint_state=JointState()
        self.got_state=False
        rospy.Subscriber(listen_prefix+'/joint_states',JointState,self.joint_state_cb)
        self.loop_rate=rospy.Rate(100.0)

    def joint_state_cb(self,joint_state):
        self.joint_state=joint_state
        self.got_state=True

    def get_joint_state(self):
        while(not self.got_state):
            self.loop_rate.sleep()
        return self.joint_state

    def send_eigen_pos_cmd(self,eig_ampl):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(0,[0,1,2,3],[],JointState(),JointTrajectory(),0,False,eig_ampl,[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def send_pos_cmd(self,q_des):
        rospy.wait_for_service(self.srv_name)
        des_js=JointState()
        des_js.position=q_des
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(0,[0,1,2,3],[],des_js,JointTrajectory(),0,False,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def send_jtraj(self,jtraj,stop_at_contact=False):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(1,[0,1,2,3],[],JointState(),jtraj,0,stop_at_contact,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def reset_tare(self,f_idx=[0,1,2,3]):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(7,f_idx,[],JointState(),JointTrajectory(),0,True,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def enable_stiffness(self,f_idx=[0,3]):
        print "currently unavailable"
        exit()
        '''
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(2,f_idx,[],JointState(),JointTrajectory(),0,True,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        '''
    def increase_stiffness(self,j_idx=[0,3],des_js=JointState()):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(4,[],j_idx,des_js,JointTrajectory(),0,True,[],[])
            return resp.reached,resp.last_des_cmd
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def eigen_increase_stiffness(self,eigen_amplitudes,js,f_idx=[0,1,2,3]):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(4,f_idx,[],js,JointTrajectory(),0,True,eigen_amplitudes,[])
            return resp.reached,resp.last_des_cmd
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        

    def disable_stiffness(self,f_idx=[0,3]):
        print "currently unavailable"
        exit()
        '''
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(4,f_idx,[],JointState(),JointTrajectory(),0,True,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        '''
    def get_smooth_traj(self,jtraj):
        rospy.wait_for_service('allegro/get_smooth_trajectory')
        traj_call=rospy.ServiceProxy('allegro/get_smooth_trajectory',GetSmoothTraj)
        max_acc=np.ones(16)*0.25
        max_vel=np.ones(16)*0.4

        resp=traj_call(jtraj,max_acc,max_vel,0.2,0.001)
        smooth_traj=resp.smooth_traj
        return smooth_traj
        
    def grasp_object(self,j_idx=[1]):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(2,[],j_idx,JointState(),JointTrajectory(),0,True,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    def approach_contact(self,j_init,f_idx,app_vec):
        des_js=JointState()
        des_js.position=j_init
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(3,f_idx,[],des_js,JointTrajectory(),0,True,app_vec,[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    def force_servo(self,f_idx,app_vec,f_mag):
        data=app_vec+f_mag
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(8,f_idx,[],JointState(),JointTrajectory(),0,True,data,[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def enable_external_control(self,external_cmd_topic):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(5,[],[],JointState(),JointTrajectory(),0,False,[],[external_cmd_topic])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    def disable_external_control(self):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(6,[],[],JointState(),JointTrajectory(),0,False,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def enable_grasp_stability(self,eigen_amplitudes,f_idx=[0,1,2,3]):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(9,f_idx,[],JointState(),JointTrajectory(),0,False,eigen_amplitudes,[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
