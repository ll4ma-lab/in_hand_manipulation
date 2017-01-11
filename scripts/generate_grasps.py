# Generate grasps given an object model
from grasp_class import grasp
import trimesh
def initialize():
    obj_mesh='../data/object_models/banana/meshes/poisson_mesh.stl'
    robot_meshes=['../data/robot_models/poisson_index.ply','../data/robot_models/poisson_middle.ply','../data/robot_models/poisson_ring.ply','../data/robot_models/poisson_thumb.ply']
    grasp_gait=grasp([0,1,2,3],robot_meshes)
    grasp_gait.load_object(obj_mesh)
    grasp_gait.object.transform([0,0, -3.14/2.0-0.5],[0.1,0.0,-0.05])
    #trimesh.io.export.export_mesh(grasp_gait.object.mesh,'banana.stl', file_type='stl') 
    grasp_gait.generate_contact_surfaces(load_from_file=True)   

    #trimesh.io.export.export_mesh(grasp_gait.contact_surfaces[0],'inter.stl', file_type='stl')
    
    grasp_gait.generate_grasp(0.001,10000)
    print grasp_gait.closest_point(0,[[0.0,0.0,0.0]])
    return grasp_gait
    
initialize()
