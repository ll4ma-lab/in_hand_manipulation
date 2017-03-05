# Contains a class which builds a trajectory model, with states as the object pose and input as the joint states.

class TrajModel:
    def __init__(self,T,dt,robot_model):
        self.T=T
        self.dt=dt
        self.robot_model=robot_model


    def get_next_state(self,x,u):
        # object pose:
        rig_fing=self.robot_model.fing
        u[rig_fing*self.robot_model.DOF:rig_fing*self.robot_model.DOF+self.robot_model.DOF]
        # Stability by maintaining the other finger poses:
        
        new_x=
        return new_x
    def get_next_input(self,x,u):
        new_input=u
        return new_input
