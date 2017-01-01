# A class to store grasps


class grasp:
    def __init__(self,fingers):
        self.fingers=fingers
        self.meshes=[[] for i in len(fingers)]

    def update_mesh(self,fing,mesh):
        self.meshes[fing]=mesh

    def generate_grasp(self,obj_mesh):

        # Place object in a bounding box

        # Find contact points 
        
        # Check stability

        # Store if within threshold
