from grasp_class import *
import numpy as np

class gaitPlanner:
    def __init__(self,initial_grasp,desired_grasp,grasp_class,psi):
        self.initial_grasp=initial_grasp
        self.desired_grasp=desired_grasp
        self.grasp=grasp_class
        self.psi=psi
    def plan_next():
        # Return finger to move next based on stability.
        
    def grasp_cost(self,grasp_points):
        # Cost between object centroid and current virtual frame
        vf=np.mean(grasp_points,axis=0)
        err=np.linalg.norm((self.object.centroid-vf)*self.psi)
        return err

    def action_cost(self,current_grasp,next_grasp,finger):
        # Cost to perform finger gaiting
        cur_vf=np.mean(current_grasp,axis=0)
        next_vf=np.mean(next_grasp,axis=0)
            
        middle_grasp=current_grasp
        del middle_grasp[finger]
        #middle_vf=np.mean(middle_grasp)

        gait_cost=self.grasp_cost(middle_grasp)
        
        # Cost to perform finger sliding
        # Interpolate poses of finger and compute stability.
        # Need to find vertices between two grasps to slide along.
