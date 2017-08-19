# A class to store grasps
import trimesh
import numpy as np
#import pcl
from collections import Iterable
import gdist
from scipy.sparse import csc_matrix
import splocs
def flatten(lis):
     for item in lis:
         if isinstance(item, Iterable) and not isinstance(item, basestring):
             for x in flatten(item):
                 yield x
         else:        
             yield item

from pathos.multiprocessing import ProcessingPool as Pool
class object:
    def __init__(self,mesh,build_g_mat=False):
        self.mesh=trimesh.load_mesh(mesh)
        self.centroid=np.mean(self.mesh.vertices,axis=0)
        if(build_g_mat):
             vert=self.mesh.vertices
             faces=self.mesh.faces.astype(np.int32)
             self.g_mat=splocs.GeodesicDistanceComputation(vert,faces)

    def transform(self,euler,translation):
        t_mat=np.identity(4)
        if(len(euler)==3):
             R=trimesh.transformations.euler_matrix(euler[0],euler[1],euler[2],'rxyz')
        else:
             R=trimesh.transformations.quaternion_matrix(euler)
        t_mat[0:3,0:3]=R[0:3,0:3]
        t_mat[0:3,3]=translation
        
        self.mesh.vertices=trimesh.transform_points(self.mesh.vertices,t_mat)
        self.centroid=np.mean(self.mesh.vertices,axis=0)
  

        
class grasp:
    def __init__(self,fingers,mesh_files,obj_mesh='NA'):
        self.fingers=fingers
        self.finger_meshes=[[] for i in range(max(fingers)+1)]
        for i in fingers:
            # Load finger meshes
            self.finger_meshes[i]=object(mesh_files[i])
        self.obj_mesh=obj_mesh
    def update_mesh(self,fing,mesh):
        self.meshes[fing]=mesh

    
    def load_object(self,obj_mesh):
        self.object=object(obj_mesh,False)

    def reset_object(self):
         self.object=object(self.obj_mesh,False)
    def contact_surface_generation(self,data):
        try:
          inter_d=data[2].intersection(data[1],engine='scad')
        except:
          print 'No intersection found'
          inter_d=[]
        return [data[0],inter_d]

    def update_contact_surface(self,idx):
         d=self.contact_surface_generation([idx,self.finger_meshes[idx].mesh,self.object.mesh])
         c_surf=d[1]

         
         if(c_surf!=[]):
              # Find contact surface only on outside of object:
              # enlarge cs:
              c_surf.apply_scale(1.2)
              # subtract :
              c_surf=c_surf.difference(self.object.mesh,engine='scad')

         self.contact_surfaces[idx]=c_surf

         
         
    def generate_contact_surfaces(self,load_from_file=False):
        
        contact_surfaces=[[] for i in range(max(self.fingers)+1)]
        cs=['f0.stl','f1.stl','f2.stl','f3.stl']
        if(not load_from_file):
            self.map=Pool().map

            # Find intersection surfaces:
            data=[]
            for i in self.fingers:
                data.append([i,self.finger_meshes[i].mesh,self.object.mesh])
            #self.contact_surface_generation(data[0])

            
            
            for res in data:
                row=self.contact_surface_generation(res)
                contact_surfaces[row[0]]=row[1]
            print contact_surfaces
            for i in self.fingers:
               if(contact_surfaces[i]!=[]):
                    # Find contact surface only on outside of object:
                    # enlarge cs:
                    contact_surfaces[i].apply_scale(1.1)
                    # subtract :
                    try:
                         data_arr=contact_surfaces[i].difference(self.object.mesh,engine='scad')
                         contact_surfaces[i]=data_arr
                         trimesh.io.export.export_mesh(contact_surfaces[i],cs[i], file_type='stl')
                    except:
                         print "Failed to find difference"+str(i)
                         trimesh.io.export.export_mesh(contact_surfaces[i],cs[i], file_type='stl')
            
        else:
            for i in self.fingers:
                contact_surfaces[i]=trimesh.load_mesh(cs[i])
        #Removing points that are inside the object:
        obj=trimesh.proximity.ProximityQuery(self.object.mesh)

        for i in self.fingers:
          if(contact_surfaces[i]!=[]):
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

    def g_dist_compute(self,idx_1,idx_2,load_mat=False):
        return self.object.g_mat(idx_1)[idx_2]

    def save_sparse(self,filename, array):
         # note that .npz extension is added automatically
         np.savez(filename, data=array.data, indices=array.indices,
                  indptr=array.indptr, shape=array.shape)

    def load_sparse(self,filename):
         # here we need to add .npz extension manually
         loader = np.load(filename + '.npz')
         return csc_matrix((loader['data'], loader['indices'], loader['indptr']),
                      shape=loader['shape'])


    def get_obj_vert(self,pt):
         # Gets the closest vertex index for a point in 3d space
         self.p_query=trimesh.proximity.ProximityQuery(self.object.mesh)
         d,idx=self.p_query.vertex(pt)
         return idx


    def get_new_sample(self,f_idx,d_idx):
         # Get vertex from contact region

         v_finger=self.contact_surfaces[f_idx].sample(1)
         
         # Get closest point on object mesh:
         c_idx=self.get_obj_vert(v_finger)
         


         return  c_idx[0],self.g_dist_compute(d_idx,c_idx[0],False)
