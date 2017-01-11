# A class to store grasps
import trimesh
import numpy as np
import pcl
from collections import Iterable
def flatten(lis):
     for item in lis:
         if isinstance(item, Iterable) and not isinstance(item, basestring):
             for x in flatten(item):
                 yield x
         else:        
             yield item

from pathos.multiprocessing import ProcessingPool as Pool
class object:
    def __init__(self,mesh):
        self.mesh=trimesh.load_mesh(mesh)
        self.centroid=np.mean(self.mesh.vertices,axis=0)
        
    def transform(self,euler,translation):
        t_mat=np.identity(4)
        R=trimesh.transformations.euler_matrix(euler[0],euler[1],euler[2],'rxyz')
        t_mat[0:3,0:3]=R[0:3,0:3]
        t_mat[0:3,3]=translation
        
        self.mesh.vertices=trimesh.transform_points(self.mesh.vertices,t_mat)
        self.centroid=np.mean(self.mesh.vertices,axis=0)
  

        
class grasp:
    def __init__(self,fingers,mesh_files):
        self.fingers=fingers
        self.finger_meshes=[[] for i in range(max(fingers)+1)]
        for i in fingers:
            # Load finger meshes
            self.finger_meshes[i]=trimesh.load_mesh(mesh_files[i])

    def update_mesh(self,fing,mesh):
        self.meshes[fing]=mesh

    
    def load_object(self,obj_mesh):
        self.object=object(obj_mesh)


    def contact_surface_generation(self,data):
        return [data[0],data[1].intersection(data[2],engine='scad')]
    
    def generate_contact_surfaces(self,load_from_file=False):
        
        contact_surfaces=[[] for i in range(max(self.fingers)+1)]
        cs=['f0.stl','f1.stl','f2.stl','f3.stl']
        if(not load_from_file):
            self.map=Pool().map

            # Find intersection surfaces:
            data=[]
            for i in self.fingers:
                data.append([i,self.finger_meshes[i],self.object.mesh])
       
        
            for res in self.map(self.contact_surface_generation,data,chunksize=1):
                row=res
                contact_surfaces[row[0]]=row[1]
            for i in self.fingers:
                trimesh.io.export.export_mesh(contact_surfaces[i],cs[i], file_type='stl')
 
        else:
            for i in self.fingers:
                contact_surfaces[i]=trimesh.load_mesh(cs[i])
        #Removing points that are inside the object:
        obj=trimesh.proximity.ProximityQuery(self.object.mesh)

        for i in self.fingers:
            points=contact_surfaces[i].vertices                
            dist=obj.signed_distance(points)
            index=[]
            vertices=[]
            face_index=[]
            bad_points=[]
            for k in range(len(dist)):
                if(dist[k]<0.000001):
                    index.append(k)
                    vertices.append(np.array(contact_surfaces[i].vertices[k]))

            contact_surfaces[i].vertices=vertices
            # Remeshing using new vertices by making faces with 3 closest points since trimesh requires faces to use any of its functions:

            tree=contact_surfaces[i].kdtree()
            d,indices=tree.query(contact_surfaces[i].vertices,3)

            faces=indices
            
           
            
            contact_surfaces[i].faces=faces
           
            contact_surfaces[i].fill_holes()

            '''
            p = pcl.PointCloud()
            p.from_array(np.array(vertices, dtype=np.float32))
            pcl.save(p,"contact_surface.ply")
            '''
        self.contact_surfaces=contact_surfaces

    def generate_grasp(self,threshold,n):
        #trimesh.io.export.export_mesh(contact_surfaces,'inter.stl', file_type='stl')
        good_grasps=[]
        # Generate grasp:
        # Pick random point from each contact surface?
        count=0
        sampling_count=0
        while(count<3 and sampling_count<n):
            sampling_count+=1
            grasp_points=[]
            #grasp_points=[[] for i in range(max(self.fingers)+1)]
            for i in self.fingers:
                grasp_points.append(np.array(self.contact_surfaces[i].vertices[np.random.randint(0,len(self.contact_surfaces[i].vertices))]))
            # Compute virtual frame origin
            vf_origin=np.mean(grasp_points,axis=0)
            # Check with object centroid
            psi=[1.0,0.0,1.0]
            err=np.linalg.norm((vf_origin-self.object.centroid)*psi)
            
            if(err<threshold):
                print vf_origin
                print self.object.centroid
                print err
                good_grasps.append(grasp_points)
                count+=1
        self.good_grasps=good_grasps
        '''
        p = pcl.PointCloud()
        p.from_array(np.array(good_grasps, dtype=np.float32))
        pcl.save(p,"grasps.ply")
        '''

    def closest_point(self,finger,point):
        '''
        Gives the closest point on the contact surface for a query point
        '''
        surface_point=trimesh.proximity.closest_point(self.contact_surfaces[finger],point)
        return surface_point


