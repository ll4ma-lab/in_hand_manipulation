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
        
        x_new=x+np.copysign((u),np.minimum(abs(u),self.max_joint_vel))*self.dt #+ some error model?

        return x_new

class inGraspSim:
    # Simulates the in-grasp manipulation assuming the object is attached to a finger.
    # Requires the robot model to simulate
    # states x=[joint_position, object_position]
    # input u=[joint_position_cmd]
    def __init__(self,robot_model,max_joint_vel,DEBUG=True):
        self.robot_model=robot_model
        self.debug=DEBUG
        #self.jointSim=robotSim(max_joint_vel)
        self.x=np.zeros(22)

    def publish_grasp(self,u):
        # Get new joint positions
        x_theta=u# + error? TODO:make this a velocity?
        # Use joint positions to get object pose
        x_obj=self.robot_model.object_pose(x_theta) # + noise
        x=np.append(x_theta,x_obj)
        return x
        
        return u
    def get_next_state(self,x,u):                
        # Get new joint positions
        #print u
        x_theta=u#self.jointSim.update_state(x[0:16],u)# + error? TODO:make this a velocity?
        # Use joint positions to get object pose
        x_obj=self.robot_model.object_pose(x_theta) # + noise
        x=np.append(x_theta,x_obj)
        return x

    def get_x_dot(self,x,u):
        x_dot=np.zeros(22)
        # theta
        x_dot[0:16]=u
        # object
        x_dot[16:]=self.robot_model.object_pose(x[:16]+u)-self.robot_model.object_pose(x[:16])
        return x_dot
        
class inGraspSimVel:
    # This model controls the robot through velocity
    # Simulates the in-grasp manipulation assuming the object is attached to a finger.
    # Requires the robot model to simulate
    # states x=[joint_position, object_position]
    # input u=[joint_position_cmd]
    # TODO: Loop rate
    def __init__(self,robot_model,max_joint_vel,DEBUG=True):
        self.robot_model=robot_model
        self.debug=DEBUG
        self.jointSim=robotSimVel(max_joint_vel,robot_model.delta_t)
        self.x=np.zeros(22)

    def publish_grasp(self,u):
        # Get new joint positions
        x_theta=u# + error? TODO:make this a velocity?
        # Use joint positions to get object pose
        x_obj=self.robot_model.object_pose(x_theta) # + noise
        x=np.append(x_theta,x_obj)
        return x
        
        return u
    def get_next_state(self,x,u):                
        # Get new joint positions
        #print u
        x_theta=self.jointSim.update_state(x[0:16],u)# + error? TODO:make this a velocity?
        # Use joint positions to get object pose
        x_obj=self.robot_model.object_pose(x_theta) # + noise
        x=np.append(x_theta,x_obj)
        return x

    def get_x_dot(self,x,u):
        x_dot=np.zeros(22)
        # theta
        x_dot[0:16]=u#*dt
        # object
        x_dot[16:]=self.robot_model.object_pose(x[:16]+u)-self.robot_model.object_pose(x[:16])
        return x_dot

        
        
