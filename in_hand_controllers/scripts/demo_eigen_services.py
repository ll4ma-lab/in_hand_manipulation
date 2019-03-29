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

from hand_client import *
#from in_hand_task_control_client import *


if __name__=='__main__':
    rospy.init_node('hand_services_demo')

    fingers=[0,1,3]
    home_js=np.array([0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,
                     -0.0,0.0,0.0,0.0,
                      1.6,0.0,0.0,0.0])

    rmp_js=np.array([0.2,0.5,0.6,0.0,
                     0.2,0.5,0.6,0.9,
                     0.2,0.5,0.6,0.0,
                     1.3,0.0,0.3,0.0])
    
    des_js=np.array([0.2,0.3,0.3,0.1,
                     0.2,0.3,0.3,0.1,
                     0.0,0.0,0.0,0.0,
                     1.6,0.0,0.3,0.0])

    # send joint position command:
    
    hand_client=handClient()
    
    raw_input("Test JPos cmd?")
    reached=hand_client.send_pos_cmd(home_js)

    # move to preshape:
    # get eigen preshape:

    raw_input("Move to eigen joint position?")
    eigen_ampl=np.zeros(5)
    eigen_ampl[1]=0.3*0.5
    eigen_ampl[4]=0.5*0.5

    des_js=hand_client.send_eigen_pos_cmd(eigen_ampl)
    
    # test approach contact:
    raw_input("Approach contact?")
    eigen_ampl=np.zeros(5)
    eigen_ampl[1]=0.6#0.3/0.5
    eigen_ampl[4]=0.5

    hand_client.approach_contact(home_js,[0,1,2,3],eigen_ampl)
    

    # Enable grasp stabilization:
    raw_input("Increase stiffness?")
    js=hand_client.joint_state
    _,js=hand_client.eigen_increase_stiffness(eigen_ampl,js)
        
    raw_input("Test JPos cmd?")
    reached=hand_client.send_pos_cmd(home_js)

    raw_input("Increase stiffness more?")
    _,js=hand_client.eigen_increase_stiffness(eigen_ampl,js)

    raw_input("Increase stiffness?")

