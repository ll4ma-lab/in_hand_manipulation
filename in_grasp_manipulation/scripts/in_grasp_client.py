import rospy
from in_grasp_manipulation.srv import InGraspManipulation,InGraspSim
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import numpy as np
import tf
import copy

# weights=[rr_orient,rr_pos,des_pose,acc,vel_lim]
# weights=[rr_orient,rr_pos,des_pose,acc,vel_lim,pose_x,pose_y,pose_z,pose_tx,pose_ty,pose_tz]

class inGraspClient:
    def __init__(self,weights=[0.00,0.5,100.0,0.1,0.6,1.0,1.0,1.0,0.1,0.1,0.1]):
        # Initialize variables:
        self.weights=weights
        self.srv_name='/in_grasp_planner/get_plan'
        
    def call_server(self,js,obj_pose,des_obj_pose,T=1,fingers=[1,3],rig_finger=3,pose_w=[1.0,1.0,1.0,1.0,1.0,1.0]):
        #j0=JointState()
        #j0.position=j_arr

        rospy.wait_for_service(self.srv_name)
        get_in_grasp_plan=rospy.ServiceProxy(self.srv_name,InGraspManipulation)


        weights=copy.deepcopy(self.weights)
        weights[0]=0.0
        weights[1]=0.0
        weights[3]=0.0
        weights[4]=100.0
        weights[5:]=pose_w
        resp=get_in_grasp_plan(js,obj_pose,des_obj_pose,weights,fingers,rig_finger,1,True, JointState(),JointTrajectory())
        
        weights=copy.deepcopy(self.weights)
        weights[3]=0.0
        weights[4]=100.0
        weights[5:]=pose_w
        
        resp=get_in_grasp_plan(js,obj_pose,des_obj_pose,weights,fingers,rig_finger,1,True, JointState(),resp.joint_traj)
        weights=copy.deepcopy(self.weights)
        
        weights[5:]=pose_w

        final_js=JointState()
        final_js.position=resp.joint_traj.points[-1].positions
        resp=get_in_grasp_plan(js,obj_pose,des_obj_pose,weights,fingers,rig_finger,T,True, final_js,resp.joint_traj)

        #return j_traj.n_js,j_traj.n_obj_pose
        return resp.joint_traj, resp.n_obj_pose.pose,resp.n_js


    def call_mpc_server(self,js,obj_pose,des_obj_pose,fingers=[0,1,2,3],rig_finger=3,jtraj=JointTrajectory(),srv_topic='/in_grasp_manipulation/mpc'):
        rospy.wait_for_service(srv_topic)
        fingers=[0,1,3]
        get_in_grasp_plan=rospy.ServiceProxy(srv_topic,InGraspManipulation)
        self.weights=[0.01,100.0,1000.0,0.0,0.4]
        j_traj=get_in_grasp_plan(js,obj_pose,des_obj_pose,self.weights,fingers,rig_finger,1,False,js,jtraj)
        return j_traj.joint_traj

    def setup_sim(self,js,obj_pose,fingers,r_f,T,var):
        rospy.wait_for_service('/in_grasp_sim/init')
        setup_in_grasp=rospy.ServiceProxy('/in_grasp_sim/init',InGraspSim)
        setup_in_grasp(js,obj_pose,fingers,r_f,T,var)
        
        
    def call_server_list(self, j_arr, obj_pose_,des_obj_pose_, weights=[1.0,1.0,1.0,1.0],fingers=[0,1,2,3],rig_finger=3,T=10,final_joints=np.array([]),initial_j_traj=JointTrajectory(),initialize_opt=True,srv_topic=''):
        j0=JointState()
        j0.position=j_arr
        j_final=JointState()
        j_final.position=final_joints
        obj_pose=Pose()
        obj_pose.position.x=obj_pose_[0]
        obj_pose.position.y=obj_pose_[1]
        obj_pose.position.z=obj_pose_[2]

        # Convert euler to quaternion:
        quaternion = tf.transformations.quaternion_from_euler(obj_pose_[3],obj_pose_[4],obj_pose_[5])
        obj_pose.orientation.x=quaternion[0]
        obj_pose.orientation.y=quaternion[1]
        obj_pose.orientation.z=quaternion[2]
        obj_pose.orientation.w=quaternion[3]

        des_obj_pose=Pose()
        des_obj_pose.position.x=des_obj_pose_[0]
        des_obj_pose.position.y=des_obj_pose_[1]
        des_obj_pose.position.z=des_obj_pose_[2]

        quaternion = tf.transformations.quaternion_from_euler(des_obj_pose_[3],des_obj_pose_[4],des_obj_pose_[5])
        des_obj_pose.orientation.x=quaternion[0]
        des_obj_pose.orientation.y=quaternion[1]
        des_obj_pose.orientation.z=quaternion[2]
        des_obj_pose.orientation.w=quaternion[3]
        
        rospy.wait_for_service(srv_topic)
        get_in_grasp_plan=rospy.ServiceProxy(srv_topic,InGraspManipulation)
        

        j_traj=get_in_grasp_plan(j0,obj_pose,des_obj_pose,weights,fingers,rig_finger,T,initialize_opt,j_final,initial_j_traj)
        #print j_traj.n_obj_pose
        if(len(j_traj.metrics)>0):
            return j_traj.joint_traj, j_traj.n_obj_pose,j_traj.obj_traj,j_traj.metrics
        else:
            return j_traj.joint_traj, j_traj.n_obj_pose,j_traj.obj_traj



'''
if __name__=='__main__':
    # in grasp data:
    j0=JointState()
    j0.position=np.ones(16)*0.4
    obj_pose=Pose()
    obj_pose.position.x=0.0
    obj_pose.position.y=0.0
    obj_pose.position.z=0.0
    obj_pose.orientation.x=0.0
    obj_pose.orientation.y=0.0
    obj_pose.orientation.z=0.0
    obj_pose.orientation.w=1.0

    des_obj_pose=Pose()
    des_obj_pose=copy.deepcopy(obj_pose)
    des_obj_pose.position.x=-0.1

    in_grasp=inGraspClient()
    j_traj=in_grasp.call_server(j0,obj_pose,des_obj_pose)
    print j_traj
'''
