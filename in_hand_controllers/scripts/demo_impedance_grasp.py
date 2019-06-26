import pickle
import rospy
import os
import sys
from external_control_client import *
from sensor_msgs.msg import JointState
from rospkg import RosPack
rp=RosPack()
rp.list()

path=rp.get_path('hand_services')+'/scripts'
sys.path.insert(0,path)
from hand_client import *

path=rp.get_path('in_hand_utils')+'/scripts'
sys.path.insert(0,path)
from utils_client import *

if __name__=='__main__':
    f_idx=[0,1,3]
    # hand client
    rospy.init_node('demo_impedance_grasp')
    
    hand_client=handClient()
    external_cl=externalHandControl()
    in_hand_utils=inHandUtils()
    home_js=np.array([0.0,0.3,0.4,0.0,
                     0.0,0.3,0.4,0.0,
                     0.0,0.3,0.4,0.0,
                      1.6,0.0,0.2,0.0])

    rmp_js=np.array([0.2,0.5,0.6,0.0,
                     0.2,0.5,0.6,0.9,
                     0.2,0.5,0.6,0.0,
                     1.3,0.0,0.3,0.0])
    des_js=np.array([0.2,0.0,0.0,0.0,
                     0.2,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,
                     1.6,0.0,0.0,0.0])
    raw_input("Test JPos cmd?")
    reached=hand_client.send_pos_cmd(home_js)
    
    hand_client.reset_tare(f_idx)
    
    raw_input('grasp object?')
    app_vec=[np.zeros(3) for i in range(len(f_idx))]
    app_vec[0]=np.array([0.0,0.0,-1.0])
    app_vec[1]=np.array([0.0,0.0,-1.0])
    #app_vec[2]=np.array([0.0,0.0,-1.0])
    app_vec[2]=np.array([0.0,0.0,1.0])

    hand_client.reset_tare(f_idx)
    
    hand_client.approach_contact(f_idx,app_vec)
    q_des=hand_client.get_joint_state()
    #print q_des
    raw_input("increase stiffness")
    j_idx=[1,2,5,6,9,10,14,15]

    _,q_des_stiff=hand_client.increase_stiffness(j_idx,q_des)
    
    # enable stiffness controller:
    raw_input("Enable impedance controller?")

    hand_client.enable_external_control('/tactile_impedance_grasp/joint_cmd')

    # call impedance controller
    mu=0.3
    external_cl.update_impedance_grasp(q_des_stiff,0.6,f_idx)
    print 'updated impedance grasp'
    
    external_cl.toggle_impedance_grasp(True)

    raw_input('disable?')
    external_cl.toggle_impedance_grasp(False)
    
    # disable external controller
    hand_client.disable_external_control()
    hand_client.send_pos_cmd(q_des.position)

    # move home
    
