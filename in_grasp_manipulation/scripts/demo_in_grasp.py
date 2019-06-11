from in_grasp_client import *
import numpy as np
import copy
import sys
from rospkg import RosPack
rp=RosPack()
rp.list()
path=rp.get_path('hand_services')+'/scripts'
sys.path.insert(0,path)

from hand_client import *

if __name__=='__main__':
    rospy.init_node('demo_in_grasp')
    f_idx=[0,1,2,3]
    hand_client=handClient()
    in_grasp_client=inGraspClient()
    home_js=np.array([0.0,0.4,0.4,0.0,
                     0.0,0.4,0.4,0.0,
                     0.0,0.4,0.4,0.0,
                      1.6,0.0,0.4,0.0])

    rmp_js=np.array([0.2,0.5,0.6,0.0,
                     0.2,0.5,0.6,0.9,
                     0.2,0.5,0.6,0.0,
                     1.3,0.0,0.3,0.0])
    
    des_js=np.array([0.2,0.3,0.3,0.1,
                     0.2,0.3,0.3,0.1,
                     0.0,0.0,0.0,0.0,
                     1.6,0.0,0.3,0.0])

    # move to preshape
    raw_input("Go to preshape?")
    hand_client.send_pos_cmd(home_js)
    
    # make contact
    # give desired fingertip joint positions and servo until contact is detected

    app_vec=[np.zeros(3) for i in range(len(f_idx))]
    app_vec[0]=np.array([0.0,0.0,-1.0])
    app_vec[1]=np.array([0.0,0.0,-1.0])
    app_vec[2]=np.array([0.0,0.0,-1.0])

    app_vec[3]=np.array([0.0,0.0,1.0])
    raw_input("Make contact?")
    hand_client.reset_tare(f_idx)

    hand_client.approach_contact(f_idx,app_vec)

    
    des_js=hand_client.get_joint_state()
    raw_input("Increase stiffness?")
    j_idx=[1,2,5,6,9,10,14,15]
    # increase stiffness
    _,des_js=hand_client.increase_stiffness(j_idx,des_js)
    
    raw_input("Increase stiffness?")
    # increase stiffness
    _,des_js=hand_client.increase_stiffness(j_idx,des_js)

    #_,des_js=hand_client.increase_stiffness(j_idx,des_js)
    raw_input("Call in-grasp planner?")
    # call in_grasp planner with desired pose
    #des_js=hand_client.get_joint_state()
    obj_pose=Pose()
    obj_pose.orientation.w=1.0
    obj_pose.position.x=0.140
    obj_pose.position.y=0.0
    obj_pose.position.z=0.01
    des_obj_pose=copy.deepcopy(obj_pose)
    des_obj_pose.position.y-=0.03
    des_obj_pose.position.x-=0.03
    des_obj_pose.position.z+=0.03
    
    
    pose_w=[1.0,0,0,0,0,0]
    t1_traj,obj_pose,_=in_grasp_client.call_server(des_js,obj_pose,des_obj_pose,T=10,fingers=f_idx,pose_w=pose_w)

    tj_traj=t1_traj

    
    pose_w=[1,0,0,0,0,0]
    des_obj_pose=copy.deepcopy(obj_pose)
    des_obj_pose.position.y-=0.02
    des_obj_pose.position.x+=0.03
    des_obj_pose.position.z+=0.03
    des_js.position=t1_traj.points[-1].positions
    t_traj,obj_pose,_=in_grasp_client.call_server(des_js,obj_pose,des_obj_pose,T=10,fingers=f_idx,pose_w=pose_w)
    tj_traj.points.extend(t_traj.points)

    
    pose_w=[0,1,0,0,0,0]
    des_obj_pose=copy.deepcopy(obj_pose)
    des_obj_pose.position.y-=0.02
    des_obj_pose.position.x+=0.03
    des_obj_pose.position.z+=0.03
    des_js.position=t_traj.points[-1].positions
    t_traj,obj_pose,_=in_grasp_client.call_server(des_js,obj_pose,des_obj_pose,T=10,fingers=f_idx,pose_w=pose_w)

    tj_traj.points.extend(t_traj.points)
    '''
    pose_w=[0,1,0,0,0,0]
    des_obj_pose=copy.deepcopy(obj_pose)
    des_obj_pose.position.y+=0.02
    des_obj_pose.position.x+=0.03
    des_obj_pose.position.z+=0.03
    des_js.position=t_traj.points[-1].positions
    t_traj,obj_pose,_=in_grasp_client.call_server(des_js,obj_pose,des_obj_pose,T=10,fingers=f_idx,pose_w=pose_w)

    tj_traj.points.extend(t_traj.points)
    
    pose_w=[0,0,1,0,0,0]
    des_obj_pose=copy.deepcopy(obj_pose)
    des_obj_pose.position.y-=0.02
    des_obj_pose.position.x+=0.03
    des_obj_pose.position.z+=0.03
    des_js.position=t_traj.points[-1].positions
    t_traj,obj_pose,_=in_grasp_client.call_server(des_js,obj_pose,des_obj_pose,T=10,fingers=f_idx,pose_w=pose_w)

    tj_traj.points.extend(t_traj.points)
    

    '''

    
    # execute on robot
    j_traj=hand_client.get_smooth_traj(tj_traj)
    
    # visualize trajectory
    hand_client.viz_joint_traj(tj_traj)

    raw_input("send traj to robot?")
    
    hand_client.send_jtraj(j_traj)
