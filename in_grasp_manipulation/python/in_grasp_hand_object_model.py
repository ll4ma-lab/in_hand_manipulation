import rospy
import numpy as np

class InGraspKinematicModel:
    def __init__(self,delta_t,T,robot_kinematic_model,fingers=[0,1,3],rigid_finger=3):
        self.robot_model=robot_kinematic_model
        
  
