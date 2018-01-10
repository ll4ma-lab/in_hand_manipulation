# Generate grasps given an object model
from grasp_class import grasp
import trimesh
import dataset_class
import numpy as np
def initialize():
    obj_mesh='../data/object_models/banana/meshes/poisson_mesh.stl'
    robot_meshes=['../data/robot_models/poisson_index.ply','../data/robot_models/poisson_middle.ply','../data/robot_models/poisson_ring.ply','../data/robot_models/poisson_thumb.ply']
    d=dataset_class.datasetAppend()
    cnt=0
    k=0
    while(k<10):
        trans=[0.1+0.05*np.random.uniform(-1.0,1.0),0.0+0.1*np.random.uniform(-1.0,1.0),
            -0.05+0.05*np.random.uniform(-1.0,1.0)]
        orient=[0+(3.14/2)*np.random.uniform(-1.0,1.0),0+(3.14/2)*np.random.uniform(-1.0,1.0), -3.14/2.0-0.5+(3.14/2)*np.random.uniform(-1.0,1.0)]
        grasp_gait=grasp([0,1,2,3],robot_meshes)
        grasp_gait.load_object(obj_mesh)
        grasp_gait.object.transform(orient,trans)
        
        emp=grasp_gait.generate_contact_surfaces(load_from_file=False,NEED_ALL_MESHES=True)
        print emp
        if(emp==True):
            k+=1
            grasp_gait.generate_grasp(0.001,5000)
            print "Found good grasps: " +str(len(grasp_gait.good_grasps))
            o_pose=np.zeros(7)
            o_pose[0:3]=trans
            o_pose[3:6]=orient
            j_angles=np.zeros(16)
            for i in range(len(grasp_gait.good_grasps)):
                cpts=np.zeros((4,3))
                for j in range(4):
                    cpts[j,:]=grasp_gait.good_grasps[i][j]
                g=[o_pose,cpts,j_angles]

                d.add_grasp('Banana',g,cnt)
                print 'Stored grasp: '+str(cnt)
                cnt+=1
initialize()
