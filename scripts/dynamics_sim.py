import numpy as np

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
'''    
class SimRelaxedRigid:
    def __init__(self,):

    def get_x_dot(self,x,u):
'''
