# Listens to allegro joint states
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
import tf
from subprocess import Popen
import subprocess
import numpy as np
import copy
import PyKDL
class robotStates:
    def __init__(self,node_name='allegro_node',publish_prefix='/allegro_hand_right',listen_prefix='/allegro_hand_right',rate=100.0,listen_aruco=False,clock_bag=False):
        rospy.init_node(node_name)
        self.rate=rospy.Rate(rate)
        self.switch_cmd=rospy.Publisher(publish_prefix+'/control_type',Int8,queue_size=10)

        self.robot_joint_cmd_pub=rospy.Publisher(publish_prefix+'/joint_cmd',
                                                     JointState, queue_size=100)

        rospy.Subscriber(listen_prefix+'/joint_states',JointState,self.joint_state_cb)
        self.clock=0
        if(listen_aruco):
            self.listener = tf.TransformListener()
        if(clock_bag):
            rospy.Subscriber('/clock',Clock,self.clock_callback)
            print 'Subscribing to clock'
        print 'Initialized robot node'
        self.loop_rate=rospy.Rate(rate)
        jc=JointState()
        jc.name = ['index_joint_0','index_joint_1','index_joint_2', 'index_joint_3',
                   'middle_joint_0','middle_joint_1','middle_joint_2', 'middle_joint_3',
                   'ring_joint_0','ring_joint_1','ring_joint_2', 'ring_joint_3',
                   'thumb_joint_0','thumb_joint_1','thumb_joint_2', 'thumb_joint_3']

        jc.position=[ 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0]
        self.jc=jc
        self.clock_bag=clock_bag

  
    def reinitialize_tf(self):
        self.listener = tf.TransformListener()
      
    def clock_callback(self,msg):
        self.clock=msg.clock#copy.deepcopy(msg.clock)

    def tighten_grasp(self,joints,robot_model,p_T_o,vel_offset):
        u=np.array(joints).copy()
        #Moves the fingertips closer to the object centroid using the Jacobian:
        for i in robot_model.fingers:
            J=robot_model.jacobian(joints[i*4:i*4+4],i)
            vel_vect=-(p_T_o*(np.matrix(vel_offset[i]).T))[0:3] #velocity direction
            u[i*4:i*4+4]=u[i*4:i*4+4]+(J.T*vel_vect*10.0).T 
        return u
    
    def tf_pose_pub(self,pose,target_frame,base_frame='palm_link'):
        tf_pose=tf.TransformBroadcaster()
        pose=pose.tolist()
        if(len(pose)==3):
            pose.extend([0.0,0.0,0.0])
        while not rospy.is_shutdown():
            for i in range(10):        
                tf_pose.sendTransform((pose[0],pose[1],pose[2]),tf.transformations.quaternion_from_euler(pose[3],pose[4],pose[5]),rospy.Time.now(),target_frame,base_frame)
                self.loop_rate.sleep()
            break
    
    def tighten_traj(self,joint_traj,robot_model,vel_gains,f_T_O,fing_T_O):
        new_traj=np.zeros((len(joint_traj),len(joint_traj[0])))
        for i in range(len(joint_traj)):
            # Find feed-forward object pose:
            p_T_thumb=robot_model.end_effector_pose(joint_traj[i][12:16],3)
            p_T_O=np.matrix(p_T_thumb)*np.matrix(f_T_O) # Palm- thumb tf * thumb- object tf
            # Compute velocity offset
            vel_offset=np.zeros((4,4))
            for j in range(len(fing_T_O)):
                vel_offset[j,0:3]=vel_gains*fing_T_O[j]
            new_joints=self.tighten_grasp(joint_traj[i],robot_model,p_T_O,vel_offset)
            new_traj[i]=new_joints.copy()
        return new_traj
    
    def tf_pose_pub_process(self,pose,target_frame,base_frame='palm_link',rate=500):
        tf_pose=tf.TransformBroadcaster()
        pose=pose.tolist()
        if(len(pose)==3):
            pose.extend([0.0,0.0,0.0])
        quat=tf.transformations.quaternion_from_euler(pose[3],pose[4],pose[5])
        #x y z qx qy qz qw frame_id child_frame_id  period_in_ms
        args=[str(pose[0]),str(pose[1]),str(pose[2]),str(quat[0]),str(quat[1]),str(quat[2]),str(quat[3]),base_frame,target_frame,str(rate)]
        cmd=['/bin/bash','rosrun','tf','static_transform_publisher']+args
        proc=Popen(cmd,stdout=subprocess.PIPE)
        return proc

    def record_bag_process(self,file_name,topics=['/tf','/camera/rgb/image_raw','/camera/rgb/camera_info','/allegro_hand_right/joint_states','/allegro_hand_right/joint_cmd']):
        cmd=['/bin/bash','rosrun','rosbag','record','-O',file_name]+topics
        proc=Popen(cmd,stdout=subprocess.PIPE)
        return proc

    def run_process(self,cmd_args):
        cmd=cmd_args
        proc=Popen(cmd,stdout=subprocess.PIPE)
        return proc
    def tf_find_T_mat(self,target_frame,base_frame):
        got_tf=False
        if (self.clock_bag):
            time=copy.deepcopy(self.clock)
        else:
            time=copy.deepcopy(rospy.Time(0))

        time=copy.deepcopy(rospy.Time(0))
        (trans_2,rot_2)= self.listener.lookupTransform(base_frame, target_frame, rospy.Time(0))
        final_pose=np.zeros(6)
        final_pose[0:3]=trans_2
        final_pose[3:6]=tf.transformations.euler_from_quaternion(rot_2)
        f_T_o=np.eye(4)
        # Match initial rotation to palm fram
        R=PyKDL.Rotation()
        R=R.RPY(final_pose[3],final_pose[4],final_pose[5])
        for k in range(3):
            for l in range(3):
                f_T_o[k,l]=R[k,l]
        f_T_o[0:3,3]=final_pose[0:3]

        got_tf=True
        #print 'tf'
    
        return f_T_o

    def tf_find_transform(self,target_frame,base_frame='palm_link'):
        got_tf=False
        if (self.clock_bag):
            time=copy.deepcopy(self.clock)
        else:
            time=copy.deepcopy(rospy.Time(0))

        time=copy.deepcopy(rospy.Time(0))
        (trans_2,rot_2)= self.listener.lookupTransform(base_frame, target_frame, rospy.Time(0))
        final_pose=np.zeros(6)
        final_pose[0:3]=trans_2
        final_pose[3:6]=tf.transformations.euler_from_quaternion(rot_2)
        got_tf=True
        #print 'tf'
    
        return final_pose

    def joint_state_cb(self,joint_state):
        self.joint_states=joint_state
        
    def publish_grasp(self,joint_pos_arr):
        self.jc.position=joint_pos_arr
        while not rospy.is_shutdown():
            for i in range(10):
                self.robot_joint_cmd_pub.publish(self.jc)
                self.loop_rate.sleep()
            break


    def publish_joint(self,joint_pos_arr):
        self.jc.position=joint_pos_arr
        #while not rospy.is_shutdown():
        for i in range(1):
            self.robot_joint_cmd_pub.publish(self.jc)
            self.loop_rate.sleep()
            #break
    def publish_joint_trajectory(self,joint_pos_traj):
        while not rospy.is_shutdown():
            for i in range(len(joint_pos_traj)):
                self.jc.position=joint_pos_traj[i]
                self.robot_joint_cmd_pub.publish(self.jc)
                self.loop_rate.sleep()
            break
        #print 'Finished running trajectory'

    def grav_comp_mode(self):
        sw_value=Int8()
        sw_value.data=2
        while not rospy.is_shutdown():
            for i in range(100):
                self.switch_cmd.publish(sw_value)
                self.rate.sleep()
            break
    def position_mode(self):
        sw_value=Int8()
        sw_value.data=0
        while not rospy.is_shutdown():
            for i in range(100):
                self.switch_cmd.publish(sw_value)
                self.rate.sleep()
            break
        

