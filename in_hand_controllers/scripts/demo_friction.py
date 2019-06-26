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



# create perception client
if __name__=='__main__':
    f_name_="friction_exp.p"
    # check for pickle file, if missing create a new file
    obj_name=raw_input("Type in object name")
    if os.path.isfile(f_name_):
        p_data = pickle.load( open( f_name_, "rb" ) )
    else:
        p_data={}
    # get object name
    f_idx=[0,1,2,3]
    rospy.init_node('demo_friction')
    external_cl=externalHandControl()
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

    # move to preshape
    hand_client=handClient()

    # get surface normal
    in_hand_utils=inHandUtils()
    sn=in_hand_utils.get_sn()
    print sn
    raw_input("Test JPos cmd?")
    reached=hand_client.send_pos_cmd(home_js)
    
    hand_client.reset_tare(f_idx)
    
    raw_input('grasp object?')
    app_vec=[np.zeros(3) for i in range(len(f_idx))]
    app_vec[0]=np.array([0.0,0.0,-1.0])
    app_vec[1]=np.array([0.0,0.0,-1.0])
    app_vec[2]=np.array([0.0,0.0,-1.0])
    app_vec[3]=np.array([0.0,0.0,1.0])

    hand_client.reset_tare(f_idx)
    
    hand_client.approach_contact(f_idx,app_vec)

    
    q_des=hand_client.get_joint_state()
    
    #print q_des
    raw_input("increase stiffness")
    j_idx=[1,2,5,6,9,10,14,15]

    _,q_des=hand_client.increase_stiffness(j_idx,q_des)

    #_,q_des=hand_client.increase_stiffness(j_idx,q_des)


    raw_input("Estimate friction?")

    mu=[]
    sn=in_hand_utils.get_sn()
    # get force pt
    force_pt=np.ravel(sn[0])*0.3
    print 'Surface normal: ',sn[0]
    #print force_pt

    control_dir=np.array([0.0,1.0,0.0])
    
    force_dir=control_dir-(np.dot(control_dir,sn[0])/np.linalg.norm(sn[0])**2)*sn[0]
    print force_dir
    #raw_input("force pt correct?")
    #force_dir=np.array([0.0,1.0,0.0])
    force_mag=0.03
    # create force dir in palm_frame

    # send to friction_estimation function

    # get friction slip vector
    s_n=np.ravel(sn[0])#np.array([0,0,-1])

    # enable external control:
    hand_client.enable_external_control('/tactile_friction_estimation/joint_cmd')
    i=0
    while i<10:
        # estimate friction
        f_slip,q_des_new=external_cl.estimate_friction(q_des,force_mag,force_pt,force_dir,f_idx=0)
        f_slip=np.ravel(f_slip)
        if(np.linalg.norm(f_slip)<0.5):
            continue
        #f_slip[0]=0.0
        #print f_slip

        i+=1
        f_slip=-1.0*np.ravel(f_slip)/np.linalg.norm(f_slip)
        print "Friction coefficient",np.tan(np.arccos(np.dot(f_slip,s_n))),np.rad2deg(np.arccos(np.dot(f_slip,s_n)))
        
        mu.append(np.tan(np.arccos(np.dot(f_slip,s_n))))
    #print mu
    print np.median(mu)#,np.mean(mu)

    p_data[obj_name]=np.ravel(mu)
    # disable external control
    hand_client.disable_external_control()
    
    # move to des_js
    hand_client.send_pos_cmd(q_des_new.position)
    
    
    raw_input("Go Home?")
    reached=hand_client.send_pos_cmd(home_js)
    pickle.dump( p_data, open( f_name_, "wb" ) )


    # store data in pickle
    
