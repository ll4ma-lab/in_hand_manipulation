import numpy as np
from kdl_allegro_model import *

class SimAllegroJoint:
    def __init__(self,dt):
        self.dt=dt
    def get_next_state(self,x,u):
        x_plus_1=x+u*self.dt
        # Check for joint limits:
        return x_plus_1

    def get_x_dot(self,x,u):
        x_dot=u
        return x_dot




class robotSim:
    def __init__(self,dt):
        # The states x=[joint states] and input u=[joint position]
        self.x=np.zeros(16)

    def update_state(self,u):
        x= u #+ some error model?
        return x
    def publish_grasp(self,u):
        # Initial grasp
        self.update_state(u)
        return x

    def publish_trajectory(self,u_arr):
        x_arr=[]
        for u in u_arr:
            x=self.update_state(u)
            x_arr.append(x)
        return x_arr

class robotSimVel:
    def __init__(self,max_joint_vel,dt,n_joints=16):
        # The states x=[joint states] and input u=[joint position]
        self.max_joint_vel=np.ravel([max_joint_vel for i in range(n_joints)])
        self.dt=dt
    def update_state(self,x,u):
        
        x_new=x+np.copysign(np.minimum(abs(u),self.max_joint_vel),u)*self.dt #+ some error model?

        return x_new

        
class inGraspSimVel:
    # This model controls the robot through velocity
    # Simulates the in-grasp manipulation assuming the object is attached to a finger.
    # Requires the robot model to simulate
    # states x=[joint_position, object_position]
    # input u=[joint_position_cmd]
    # TODO: Loop rate
    def __init__(self,robot_model,max_joint_vel,joint_noise=False,obj_noise=False,DEBUG=False):
        self.robot_model=robot_model
        self.debug=DEBUG
        self.max_joint_vel=max_joint_vel
        self.jointSim=robotSimVel(max_joint_vel,robot_model.delta_t)
        self.x=np.zeros(22)
        self.joint_noise=joint_noise
        self.obj_noise=obj_noise
    def publish_grasp(self,u):
        # Get new joint positions
        x_theta=u# + error? TODO:make this a velocity?
        # Use joint positions to get object pose
        x_obj=self.robot_model.object_pose(x_theta) # + noise
        x=np.append(x_theta,x_obj)
        return x        
        
    def get_next_state(self,x,u,err=False):                
        # Get new joint positions
        #print u
        x_theta=self.jointSim.update_state(x[0:16],u)
        if (self.joint_noise==True and err==True):
            # Adding Gaussian noise:
            # normal(mean,std. deviation, array size)
            x_theta+=np.random.normal(0,1.0,len(x_theta))*0.005
        # Use joint positions to get object pose
        x_obj=x[16:]+(self.robot_model.object_pose(x_theta)-x[16:])*self.robot_model.delta_t
        if(self.obj_noise==True and err==True):
            # Adding Gaussian noise:
            # normal(mean,std. deviation, array size)
            x_obj+=np.random.normal(0,1.0,len(x_obj))*0.0005
        x=np.append(x_theta,x_obj)
        return x

    def get_x_dot(self,x,u):
        x_dot=np.zeros(22)
        # theta
        x_dot[0:16]=u
        # Since the 
        #x_dot[12:16]+=

        # object
        x_theta_plus_1=self.jointSim.update_state(x[0:16],u)# + error? TODO:make this a velocity?
        
        x_dot[16:]=self.robot_model.object_pose(x_theta_plus_1)-x[16:]
        return x_dot

        
        
