# This file creates an instance of the allegro robot with FK and Jacobian using KDL.
import rospy
from rospkg import RosPack
import handModel
import numpy as np
from numpy import cos,sin
from numpy.linalg import inv
from numpy.linalg import pinv
import PyKDL
class allegroKinematics:
    def __init__(self,delta_t=1.0/60.0,T=10,fingers=[0,3],fing=3):
        ''' delta_t= timestep
            T= total time steps
            fingers=[0-4],0=index,1=middle,2=ring,3=thumb        
            The model assumes all fingers have same number of joints.
        '''
        self.n=6
        self.m=4*4
        self.T=T
        self.delta_t=delta_t
        self.fingers=np.array(fingers)
        self.fing=3
        rp = RosPack()
        packages = rp.list()
        path = rp.get_path('urlg_robots_description')
        path='../urdf/allegro.urdf'
        #path= path + '/urdf/allegro_right/allegro_hand_description_right.urdf'
        self.kdlModel=handModel.HandSimpleModel(path)
        low_bounds=[]
        up_bounds=[]
        for i in xrange(4):
            low_bounds+=list(self.kdlModel.finger_chains[i].get_joint_limits()[0])
            up_bounds+=list(self.kdlModel.finger_chains[i].get_joint_limits()[1])
        self.bounds=np.array([low_bounds,up_bounds])
        #print self.bounds
        self.DOF=4
        self.FINGERS=4
   
    
    def object_frame_init(self,D,joints,finger=3):
        t_T_f=self.end_effector_pose(joints[finger*4:finger*4+4],finger)
        t_R_f=t_T_f[0:3,0:3]
        f_R_t=inv(t_R_f)
       
        f_T_o=np.eye(4)
        f_T_o[0:3,0:3]=f_R_t
        f_T_o[0:3,3]=D

        return np.matrix(f_T_o)
        
    def update_object_frame(self,f_T_o,joints,finger=3):
        t_T_f=self.end_effector_pose(joints[finger*4:finger*4+4],finger)
        object_frames=[[np.eye(4)]for i in range(4)]
        inv_object_frames=[[np.eye(4)]for i in range(4)]
        object_frames[finger]=f_T_o
        inv_object_frames[finger]=inv(f_T_o)
        fingers=self.fingers[self.fingers!=finger]
        for i in fingers:            
            # Creating frames for other fingers:
            i_T_o=inv(np.matrix(self.end_effector_pose(joints[i*4:i*4+4],i)))*np.matrix(t_T_f)*np.matrix(f_T_o)
            object_frames[i]=np.matrix(i_T_o)
            inv_object_frames[i]=inv(np.matrix(i_T_o))

        finger_dist=[[0.0,0.0,0.0] for i in range(self.FINGERS)]
        
        # Yaw angle constant:
        yaw_rig=np.zeros((self.FINGERS,self.FINGERS))
        orient_rig=[np.zeros(3) for i in range(self.FINGERS)]

        u=joints
        i=finger
        for k in self.fingers:
            T_mat=(self.end_effector_pose(u[i*self.DOF:i*self.DOF+self.DOF],i))*self.end_effector_pose(u[k*self.DOF:k*self.DOF+self.DOF],k)    
            diff=np.ravel(T_mat[0:3,3])
            finger_dist[k]=diff
            a=self.end_effector_pose_array(u[i*self.DOF:i*self.DOF+self.DOF],i)[3:6]
            b=self.end_effector_pose_array(u[k*self.DOF:k*self.DOF+self.DOF],k)[3:6]
            diff_yaw=a-b
            diff_orient=a-b
            yaw_rig[i,k]=diff_yaw[1]
            orient_rig[k]=diff_orient
        self.finger_dist=finger_dist
        self.inv_object_frames=inv_object_frames
        self.yaw_rig=yaw_rig
        self.orient_rig=orient_rig
        return object_frames

    def add_object_frame(self,D,joints,finger=3):
        # Adds object frame at a distance D from finger 0 and creates frame transformations to this frame from other fingertips
        f_T_o=self.object_frame_init(D,joints,finger) 
        t_T_f=self.end_effector_pose(joints[finger*4:finger*4+4],finger)
        object_frames=[[np.eye(4)]for i in range(4)]
        inv_object_frames=[[np.eye(4)]for i in range(4)]
        object_frames[finger]=f_T_o
        inv_object_frames[finger]=inv(f_T_o)
        fingers=self.fingers[self.fingers!=finger]
        for i in fingers:            
            # Creating frames for other fingers:
            i_T_o=inv(np.matrix(self.end_effector_pose(joints[i*4:i*4+4],i)))*np.matrix(t_T_f)*np.matrix(f_T_o)
            object_frames[i]=np.matrix(i_T_o)
            inv_object_frames[i]=inv(np.matrix(i_T_o))

        finger_dist=[[0.0,0.0,0.0] for i in range(self.FINGERS)]
        
        # Yaw angle constant:
        yaw_rig=np.zeros((self.FINGERS,self.FINGERS))
        orient_rig=[np.zeros(3) for i in range(self.FINGERS)]
        u=joints
        i=finger
        for k in self.fingers:
            T_mat=(self.end_effector_pose(u[i*self.DOF:i*self.DOF+self.DOF],i))*self.end_effector_pose(u[k*self.DOF:k*self.DOF+self.DOF],k)    
            diff=np.ravel(T_mat[0:3,3])
            finger_dist[k]=diff
            a=self.end_effector_pose_array(u[i*self.DOF:i*self.DOF+self.DOF],i)[3:6]
            b=self.end_effector_pose_array(u[k*self.DOF:k*self.DOF+self.DOF],k)[3:6]
            diff_yaw=a-b
            diff_orient=a-b
            yaw_rig[i,k]=diff_yaw[1]
            orient_rig[k]=diff_orient
        self.finger_dist=finger_dist
        self.inv_object_frames=inv_object_frames
        self.yaw_rig=yaw_rig
        self.orient_rig=orient_rig
        return object_frames
  
    
    def object_frame_init_trinkle(self,joints):
        # Get initial object pose
        t_X_O=self.object_pose(joints)
        t_T_O=np.eye(4)
        # Match initial rotation to palm fram
        R=PyKDL.Rotation()
        R=R.RPY(t_X_O[3],t_X_O[4],t_X_O[5])
        for k in range(3):
            for l in range(3):
                t_T_O[k,l]=R[k,l]
        t_T_O[0:3,3]=t_X_O[0:3]
        # Built object contact point frames
        contact_frames=[[np.eye(4)]for i in range(4)]
        t_T_f=np.eye(4)
        for finger in self.fingers:
            t_T_f[0:3,3]=self.end_effector_pose(joints[finger*4:finger*4+4],finger)[0:3,3].T
            # Frame from object to finger tip
            contact_frames[finger]=np.matrix(inv(t_T_O))*np.matrix(t_T_f)
            

        self.contact_frames=contact_frames

    '''
    def contact_points(self,joints,obj_frames):
        #finger=0
        for finger in range(4):        
            t_T_0=self.end_effector_pose(joints[finger*4:finger*4+4],finger)
            t_T_o=t_T_0*inv(obj_frames[finger])
            print t_T_o
        return 0
    '''
    def contact_points(self,des_obj_mat):
       
        des_contact_frames=[[np.eye(4)]for i in range(4)]
        des_contact_points=[[np.zeros(3)] for i in range(4)]
        for i in self.fingers:
            cp=np.zeros(4)
            cp[3]=1.0
            cp[0:3]=np.ravel(self.contact_frames[i][0:3,3])
            des_contact_frames[i]=des_obj_mat*self.contact_frames[i]
            des_contact_points[i]=(np.matrix(des_obj_mat)*np.matrix(cp).T)[0:3]
        self.des_contact_frames=des_contact_frames
        self.des_contact_points=des_contact_points

    def sampling_contact_points(self,des_obj_pose,init_obj_pose):
        T=self.T

        time_contacts=[]
        for k in range(T):
            sample_pose=init_obj_pose+(des_obj_pose-init_obj_pose)*float(k)/T
            # Building transformation matrix:
            x_des_mat=np.eye(4)
            x_des_mat[0:3,3]=sample_pose[0:3]
            R=PyKDL.Rotation.RPY(sample_pose[3],sample_pose[4],sample_pose[5])
            for i in range(3):
                for j in range(3):
                    x_des_mat[i,j]=R[i,j]
        

            des_contact_frames=[[np.eye(4)]for i in range(4)]
            des_contact_points=[[np.zeros(3)] for i in range(4)]
            
            for i in self.fingers:
                cp=np.zeros(4)
                cp[3]=1.0
                cp[0:3]=np.ravel(self.contact_frames[i][0:3,3])
                des_contact_frames[i]=x_des_mat*self.contact_frames[i]
                des_contact_points[i]=(np.matrix(x_des_mat)*np.matrix(cp).T)[0:3]
            time_contacts.append(des_contact_frames)
        
        self.time_contacts=time_contacts

    def object_pose(self,joints,finger=3,get_quat=False):
        obj_T=self.end_effector_pose(joints[finger*4:finger*4+4],finger)*self.object_frames[finger]
        if get_quat:
            obj_pose=np.zeros(7)
        else:
            obj_pose=np.zeros(6)
        obj_pose[0:3]=np.ravel(obj_T[0:3,3])
        R=PyKDL.Rotation(obj_T[0,0],obj_T[0,1],obj_T[0,2],obj_T[1,0],obj_T[1,1],obj_T[1,2],obj_T[2,0],obj_T[2,1],obj_T[2,2])
        if get_quat:
            obj_pose[3:7]=np.array(R.GetQuaternion())
        else:
            obj_pose[3:6]=np.array(R.GetRPY())
        return obj_pose

   # def object_pose_jacobian(self,joints,finger=0):
    def object_T(self,joints,finger=3):
        obj_T=self.end_effector_pose(joints[finger*4:finger*4+4],finger)*self.object_frames[finger]     
        return obj_T
    
    def object_finger_diff(self,obj_mat,finger=3):
        # Takes in object matrix and finger angles and gives the desired object pose with respect to the finger
        des_f=np.matrix(obj_mat)*np.matrix(self.inv_object_frames[finger])
        R=PyKDL.Rotation(des_f[0,0],des_f[0,1],des_f[0,2],des_f[1,0],des_f[1,1],des_f[1,2],des_f[2,0],des_f[2,1],des_f[2,2])
        des_finger_pose=np.zeros(6)
        des_finger_pose[0:3]=des_f[0:3,3].ravel()
        des_finger_pose[3:6]=R.GetRPY()
        return des_finger_pose

    def finger_rigid_pose(self,u,f0,f1):
        # gives the pose of f1 given f0 in the reference frame of rigid f1
        f0_frame=self.end_effector_pose(u[f0*4:f0*4+4],f0)
        f1_frame=np.matrix(f0_frame)*np.matrix(self.object_frames[f0])*np.matrix(self.inv_object_frames[f1])
        '''        
        inv_frame=inv(np.matrix(f1_frame_2d))
        f1_frame=inv_frame*self.end_effector_pose(u[f1*4:f1*4+4],f1)
        '''
        f1_pose=np.zeros(6)
        f1_pose[0:3]=f1_frame[0:3,3].ravel()
        R=PyKDL.Rotation(f1_frame[0,0],f1_frame[0,1],f1_frame[0,2],f1_frame[1,0],f1_frame[1,1],f1_frame[1,2],
                         f1_frame[2,0],f1_frame[2,1],f1_frame[2,2])
        f1_pose[3:6]=R.GetRPY()
        return f1_pose
    def end_effector_pose_array(self,u,EE_index):
        T=self.kdlModel.FK(u,EE_index)
        pose=np.zeros(6)
        R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],T[1,0],T[1,1],T[1,2],T[2,0],T[2,1],T[2,2])
        pose[3:6]=R.GetRPY()
        pose[0:3]=T[0:3,3].ravel()
        return pose

    def predict_T(self,x0,u):
        T=self.T
        x_n=np.array([[0.0 for z in range(self.m)] for i in range(T)])
        x_n[0,:]=x0
        for i in range(1,T):
            x_n[i,:]=self.predict(x_n[i-1,:],np.ravel(u[i,:]))
        return x_n

    def predict(self,x,u):
        x_k_plus_1=np.array(u)
        return x_k_plus_1

    def end_effector_pose(self,u,EE_index=0):

        T=self.kdlModel.FK(u,EE_index)
        return T

    def end_effector_pose_T(self,x0,u,EE_index=0):
        T=self.T
        x_n=np.array([[0.0 for z in range(3)] for i in range(T)])
        x_n[0,:]=x0
        for i in range(1,T):
            x_n[i,:]=np.ravel(self.end_effector_pose(np.ravel(u[i,0:4]),EE_index)[0:3,3])
        return x_n

    def jacobian(self,u,EE_index=0):
        J=self.kdlModel.Jacobian(u,EE_index)[0:3,:]
        return J
    def jacobian_orient(self,u,EE_index=0):
        J=self.kdlModel.Jacobian(u,EE_index)[3:6,:]
        return J
    def jacobian_full(self,u,EE_index=0):
        J=self.kdlModel.Jacobian(u,EE_index)
        return J

    def predict_xdot(self,x,u):
        # given the current object pose and the new input, find the velcity
        #des_finger_pose=np.ravel(self.linear_finger_poses[t_step])
        x_pose=self.end_effector_pose_array(u[3*4:3*4+4],3)
        x_dot=(x-x_pose)/self.delta_t
        x_new=x_dot
        return x_new

    # Adding functions to compute object_pose_cost
    def init_object_offset(self,joints,finger_arr):
        # Initializes the offsets to the fingers from the object in the palm frame
        self.obj_offset=np.zeros((self.FINGERS,6))

        for i in finger_arr:            
            f_T_0=self.object_frames[i]
            obj_pose_mat=self.end_effector_pose(joints[i*4:i*4+4],i)*f_T_0
            obj_pose_arr=np.zeros(6)
            obj_pose_arr[0:3]=obj_pose_mat[0:3,3].T
            R=PyKDL.Rotation(obj_pose_mat[0,0],obj_pose_mat[0,1],obj_pose_mat[0,2],obj_pose_mat[1,0],obj_pose_mat[1,1],obj_pose_mat[1,2],obj_pose_mat[2,0],obj_pose_mat[2,1],obj_pose_mat[2,2])
            obj_pose_arr[3:6]=np.array(R.GetRPY())
            self.obj_offset[i]=obj_pose_arr-self.end_effector_pose_array(joints[i*4:i*4+4],i)

    def min_object_pose(self,u,finger_arr):
        A=np.zeros((len(finger_arr)*6,6))
        B=np.zeros((len(finger_arr)*6,1))
        for i in range(len(finger_arr)):
            finger=finger_arr[i]
         
            B[i*6:i*6+6,0]=self.end_effector_pose_array(u[finger*4:finger*4+4],finger)
            A[i*6:i*6+6,0:6]=np.eye(6)

        X=pinv(A)*np.matrix(B)
        
        return np.ravel(X)

    
